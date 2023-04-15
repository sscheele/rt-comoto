# import Base
# Base.copy(c::GeneralCost) = c
import RobotDynamics;

RobotDynamics.@autodiff struct CompoundCost{n,m} <: TO.CostFunction
    cost_fns::AbstractVector{Function}
    weights::AbstractVector{Float64}
end
function RobotDynamics.evaluate(cost::CompoundCost, x, u)
    cost_weights = zip(cost.weights, cost.cost_fns);
    # total = 0.;
    # for (c1, c2) in cost_weights
    #     fn_val = c2(x, u);
    #     if length(string(fn_val)) < 20
    #         println(fn_val)
    #     end
    #     total += c1*fn_val;
    # end
    # println("Total: ", total);
    # return total;
    components = map(z -> z[1]*z[2](x, u), cost_weights);
    return sum(components);
end
RobotDynamics.state_dim(::CompoundCost{n,<:Any}) where n = n;
RobotDynamics.control_dim(::CompoundCost{<:Any,m}) where m = m;
function RobotDynamics.gradient!(fun::CompoundCost, grad, x, u)
    f = z -> RobotDynamics.evaluate(fun, z[1:length(x)], z[length(x)+1:end]);
    ForwardDiff.gradient!(grad, f, [x;u], fun.gradcfg);
    return nothing;
end
function RobotDynamics.hessian!(fun::CompoundCost, grad, x, u)
    f = z -> RobotDynamics.evaluate(fun, z[1:length(x)], z[length(x)+1:end]);
    ForwardDiff.hessian!(grad, f, [x;u], fun.hesscfg);
    return nothing;
end



# function RobotDynamics.evaluate(cost::CompoundCost, x, u)
#     total = 0.;
#     for i=1:length(cost.weights)
#         total += cost.weights[i] + cost.cost_fns[i](x, u);
#     end
#     total
# end


@inline function sq_norm(x::AbstractVector{T}) where T
    dot(x, x)::T
end

"""
Returns the legibility cost for a given timestep

jacobian: function that accepts a joint state and returns the Jacobian matrix J
dt: the time allotted to one timestep (eg, 1.0 sec/timestep)
curr_timestep: the current timestep
total_timesteps: the total number of timesteps in the trajectory
goalset: 3xN matrix possible goals - first element (goalset[:,1]) is "true goal"
start: initial cartesian position
ee_fn: end effector fk function
"""
function legibility_costfn(x::AbstractVector, u::AbstractVector, dt::Float64, curr_timestep::Int, total_timesteps::Int, goalset::AbstractMatrix, start::AbstractVector, ee_fn::Function)
    # exp(-C(s->q) - C*(q->g))/exp(-C*(s->g))
    # = exp(-C(s->q) - C*(q->g) + C(s->g))
    cart_x = translation(ee_fn(x));
    n_goals = size(goalset)[2];
    # create matrices that we can subtract from goalset
    n_dims = length(start)
    start_rep, x_rep = reshape(start, (n_dims, 1)), reshape(cart_x, (n_dims, 1))
    start_goal_velset = (goalset .- start_rep)/(dt * (total_timesteps - 1)); # TODO: validate -1
    nom_goal_velset = (goalset .- x_rep)/(dt * (total_timesteps - curr_timestep));

    goal_weight = exp(-sq_norm(nom_goal_velset[:,1]) + sq_norm(start_goal_velset[:,1]));
    total_weight = goal_weight + 0.0001; # add eps for numerical stability
    for i = 2:n_goals
        total_weight += exp(-sq_norm(nom_goal_velset[:,i]) + sq_norm(start_goal_velset[:,i]));
    end
    leg = goal_weight/total_weight    
    if isnan(leg)
        println("NaN in legibility cost");
    end
    1-leg
end

"""
Returns an array of length total_timesteps of legibility cost functions
"""
function get_legibility_costs(info::ComotoProblemInfo)
    n, m = info.state_dims, info.ctrl_dims
    ret_val = Function[];
    cart_start = translation(info.eef_fk(info.joint_start));
    for i = 0:(info.n_timesteps - 2)
        append!(ret_val, [(x, u) -> legibility_costfn(x, u, info.dt, i, info.n_timesteps, info.goal_set, cart_start, info.eef_fk)]);
    end
    ret_val
end

"""
Returns a vector of length n_timesteps of joint velocity costs
"""
function get_jointvel_costs(info::ComotoProblemInfo)
    n, m = info.state_dims, info.ctrl_dims
    ret_val = Function[];
    for i = 1:(info.n_timesteps - 1)
        append!(ret_val, [(x, u) -> dot(u,u)]);
    end
    ret_val
end

"""
Returns the visibility cost at a given timestep, defined as the angle
between the head-object axis and the head-eef axis

eef_pos: cartesian eef position
object_pos: cartesian object position
head_pos: cartesian head position
"""
function visibility_costfn(eef_pos::AbstractVector, object_pos::AbstractVector, head_pos::AbstractVector)
    # return 0;
    obj_axis = object_pos .- head_pos;
    eef_axis = eef_pos .- head_pos;
    # add a small epsilon to denom for numerical stability
    ret_val = acos(dot(eef_axis, obj_axis)/(norm(eef_axis)*norm(obj_axis) + 0.001));
    ret_val
end

"""
Returns a vector of length n_timesteps of visibility costs
"""
function get_visibility_costs(info::ComotoProblemInfo)
    #n::Int, m::Int, head_traj::AbstractMatrix, object_pos::AbstractVector, ee_posfn::Function
    n, m = info.state_dims, info.ctrl_dims
    ret_val = Function[];
    for i = 1:info.n_timesteps
        append!(ret_val, [(x, u) -> visibility_costfn(translation(info.eef_fk(x)), info.object_pos, info.head_traj[:,i])]);
    end
    ret_val
end

"""
Return the distance between each robot joint and its closest human joint

cart_joints: 3xN_ROBOT_JOINTS array of cartesian robot joint positions
human_pos: 3xN_HUMAN_JOINTS array of human joint positions
"""
function jointwise_distance(cart_joints::AbstractArray, human_pos::AbstractArray)
    ret_val = zeros(size(cart_joints[2]))
    for rjoint_idx = 1:size(cart_joints)[2]
        diff_arr = human_pos .- cart_joints[:,rjoint_idx];
        dists = [norm(diff_arr[:,x]) for x=1:size(human_pos[2])]
        ret_val[rjoint_idx] = minimum(dists)
    end
    ret_val
end

"""
Return the distance cost for a given timestep

cart_joints: 3xN_ROBOT_JOINTS array of cartesian robot joint positions
human_pos: 3xN_HUMAN_JOINTS array of human joint positions
human_vars: 3x3xN_HUMAN_JOINTS array of human joint covariance matrices
"""
function distance_costfn(cart_joints::AbstractArray, human_pos::AbstractArray, human_vars::AbstractArray)
    # return 0.0*cart_joints[1]
    n_robot_joints = size(cart_joints)[2]
    n_human_joints = size(human_pos)[2]
    
    cost_total = 0.0;
    for rjoint_idx = 5:n_robot_joints
        for hjoint_idx = 1:n_human_joints
            diff = abs.(cart_joints[:,rjoint_idx] - human_pos[:,hjoint_idx]);
            curr_cost_inv = diff'*inv(human_vars[:,:,hjoint_idx])*diff;
            cost_total += 1/(curr_cost_inv + 1e-4);
        end
    end
    if isnan(cost_total)
        println("NaN in distance cost");
    elseif cost_total < 0
        println("Negative distance cost!");
        exit();
    end
    return cost_total;
end

"""
Returns a vector of length n_timesteps of distance costs
"""
function get_distance_costs(info::ComotoProblemInfo)
    n, m = info.state_dims, info.ctrl_dims
    ret_val = Function[];
    
    for i = 1:info.n_timesteps
        curr_hpos = info.human_traj[:,:,i];
        curr_hvars = info.human_vars_traj[:,:,:,i];
        append!(ret_val, [(x, u) -> distance_costfn(info.full_fk(x), curr_hpos, curr_hvars)]);
    end
    ret_val
end

"""
Returns the nominal cost at a timestep, defined as the square cartesian distance between
end effector positions of the nominal and actual trajectories

cart_eef: cartesian end effector position (actual)
goal_eef: cartesian end effector position (nominal)
"""
function nominal_costfn(cart_eef::AbstractVector, goal_eef::AbstractVector)
    ret_val = sq_norm(cart_eef .- goal_eef);
    ret_val
end

"""
Returns a vector of length n_timesteps of nominal costs
"""
function get_nominal_costs(info::ComotoProblemInfo)
    n, m = info.state_dims, info.ctrl_dims
    ret_val = Function[];

    nom_eef_traj = [translation(info.eef_fk(info.nominal_traj[:, i])) for i=1:info.n_timesteps];
    for i = 1:info.n_timesteps
        append!(ret_val, [(x, u) -> nominal_costfn(translation(info.eef_fk(x)), 
            nom_eef_traj[i])]);
    end
    ret_val
end


# function pose_costfn(cart_eef_trans::Transform3D, cart_goal::Transform3D)
#     posn_err = sq_norm(translation(cart_eef_trans) .- translation(cart_goal));
#     curr_orient = UnitQuaternion(rotation(cart_eef_trans));
#     goal_orient = UnitQuaternion(rotation(cart_goal));
#     orient_err = curr_orient.w*goal_orient.w + curr_orient.x*goal_orient.x + curr_orient.y*goal_orient.y + curr_orient.z*goal_orient.z;
#     orient_err = 1 - orient_err^2;
#     sqrt(posn_err) + orient_err
# end

# function get_pose_costs(info::ComotoProblemInfo)
#     n, m = info.state_dims, info.ctrl_dims
#     ret_val = Function[];
#     goal_pos = info.eef_fk(info.joint_target);

#     for i = 1:info.n_timesteps
#         append!(ret_val, [(x, u) -> pose_costfn(info.eef_fk(x), goal_pos)]);
#     end
#     ret_val
# end

function pose_costfn(joint_pos::AbstractVector, joint_goal::AbstractVector, u::AbstractVector, ts_left::Int)
    u_nom = joint_goal .- joint_pos;
    max_mag = maximum(abs.(u_nom));
    if max_mag > 0.2
        u_nom = u_nom ./ max_mag;
        u_nom = u_nom .* 0.2; # 0.2 seems like a good joint velocity
    end
    
    return sqrt(sq_norm(u_nom - u));    
    
    # u_nom_mag = sqrt(sq_norm(u_nom));   
    # dist_scale = (1 - 2.71828^(-5.0 * u_nom_mag));     
    # return (-1. / u_nom_mag) * dist_scale * dot(u_nom, u);
end

function get_pose_costs(info::ComotoProblemInfo)
    n, m = info.state_dims, info.ctrl_dims
    ret_val = Function[];

    for i = 1:(info.n_timesteps - 1)
        push!(ret_val, (x, u) -> pose_costfn(x, info.end_goal, u, info.n_timesteps - i + 1));
    end
    ret_val
end

"""
Returns a vector of length n_timesteps of comoto costs (combination of legibility,
visibility, distance, nominal, and joint velocity costs). Costs are weighted
according to the weights vector.
Weights: leg, vis, dist, nom, vel
"""
function get_comoto_costs(info::ComotoProblemInfo, weights::AbstractVector{Float64})
    leg_costs = get_legibility_costs(info);
    vis_costs = get_visibility_costs(info);
    dist_costs = get_distance_costs(info);
    nom_costs = get_nominal_costs(info);
    vel_costs = get_jointvel_costs(info);

    ccost(f, w) = CompoundCost{info.state_dims,info.ctrl_dims}(f, w)
    
    all_costs = [leg_costs, vis_costs, dist_costs, nom_costs, vel_costs];
    final_costs = CompoundCost{info.state_dims,info.ctrl_dims}[];
    for i = 1:(info.n_timesteps-1)
        append!(final_costs, [ccost([c[i] for c in all_costs], weights)]);
    end
    last_ts_costs = [];
    last_ts_weights = [];
    for i=1:length(all_costs)
        if length(all_costs[i]) == info.n_timesteps
            append!(last_ts_costs, [all_costs[i][end]])
            append!(last_ts_weights, [weights[i]])
        end
    end
    append!(final_costs, [ccost(last_ts_costs, last_ts_weights)]);
    final_costs
end

"""
Returns a vector of length n_timesteps of sliding window costs (comoto plus goal). 
Costs are weighted according to the weights vector:
Weights: leg, vis, dist, nom, vel, goal
"""
function get_sliding_costs(info::ComotoProblemInfo, weights::AbstractVector)
    leg_costs = get_legibility_costs(info);
    vis_costs = get_visibility_costs(info);
    dist_costs = get_distance_costs(info);
    nom_costs = get_nominal_costs(info);
    vel_costs = get_jointvel_costs(info);
    goal_costs = get_pose_costs(info);

    ccost(f, w) = CompoundCost{info.state_dims,info.ctrl_dims}(f, w)
    
    all_costs = [leg_costs, vis_costs, dist_costs, nom_costs, vel_costs, goal_costs];
    final_costs = CompoundCost{info.state_dims,info.ctrl_dims}[];
    for i = 1:(info.n_timesteps-1)
        append!(final_costs, [ccost([c[i] for c in all_costs], weights)]);
    end
    last_ts_costs = [];
    last_ts_weights = [];
    for i=1:length(all_costs)
        if length(all_costs[i]) == info.n_timesteps
            append!(last_ts_costs, [all_costs[i][end]])
            append!(last_ts_weights, [weights[i]])
        end
    end
    append!(final_costs, [ccost(last_ts_costs, last_ts_weights)]);
    final_costs
end