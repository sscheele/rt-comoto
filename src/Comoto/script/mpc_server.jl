import Pkg; Pkg.activate("..");
 
push!(LOAD_PATH, "../")
using Comoto: RobotInfo, ComotoProblemInfo, ManipVelCtrl, ComotoParameters;
using Comoto: get_probinfo, get_sliding_costs, PosEECons;
using Comoto.trajectory_msgs.msg;
using Comoto.julia_comoto.srv;
using LinearAlgebra;
using StaticArrays;
using RobotOS;
using Altro: ALTROSolver, SolverOptions, solve!;
 
using TrajectoryOptimization;
const TO = TrajectoryOptimization;

# PLAN_DT is the timestep length for planning
const PLAN_DT = 0.4
# N_HUMAN_JOINTS is the number of human joints
const N_HUMAN_JOINTS = 11

float(x::Duration) = x.secs + (x.nsecs/1e9)

"""
Linearly interpolates a value for f(x) if f(x1)=y1 and f(x2)=y2
Assumes x1<x<x2
"""
function linear_interp(x, x1, x2, y1, y2)
    progress = (x - x1)/(x2 - x1);
    return progress*y2 + (1-progress)*y1;
end

"""
ROS callback - replan the robot trajectory based on
a new predicted human trajectory
"""
function replan(req::SolveRequest, 
                robot_info::RobotInfo,
                base_probinfo::ComotoProblemInfo,
                opts::SolverOptions)
    println("Received request!");
    t_start = time();
    n_timesteps = length(req.pred_traj.points);
    joint_start = SVector{robot_info.dof}(req.joint_start);
    joint_target = SVector{robot_info.dof}(
        req.nom_traj.points[end].positions
    );
    object_set = SArray{Tuple{3, Int(floor(length(req.object_set)/3))}}(
        req.object_set
    );
    object_pos = object_set[:,1];
    end_goal = SVector{robot_info.dof}(req.end_goal)
    
    # params = ComotoParameters(
    #     joint_start,
    #     joint_target,
    #     n_timesteps,
    #     PLAN_DT,
    #     object_set,
    #     end_goal
    # )

    # copy the human and nominal trajectories into StaticArrays of
    # appropriate dimensions
    human_traj_mut = zeros(3, N_HUMAN_JOINTS, n_timesteps);
    nom_traj_mut = zeros(robot_info.dof, n_timesteps);
    for i=1:n_timesteps
        nom_traj_mut[:,i] = req.nom_traj.points[i].positions;
        human_traj_mut[:,:,i] = reshape(
            req.pred_traj.points[i].positions,
            3, N_HUMAN_JOINTS
        );
    end
    human_traj = SArray{Tuple{3,N_HUMAN_JOINTS,n_timesteps}}(human_traj_mut);
    nom_traj = SArray{Tuple{robot_info.dof,n_timesteps}}(nom_traj_mut);

    subprob_info = ComotoProblemInfo(
        base_probinfo.joint_tree,
        base_probinfo.eef_fk,
        base_probinfo.full_fk,
        base_probinfo.ctrl_dims,
        base_probinfo.state_dims,
        n_timesteps,
        PLAN_DT,
        object_pos,
        object_set,
        base_probinfo.human_goal, # TODO check this
        human_traj,
        human_traj[:,6,:], # head traj
        repeat(0.05*Matrix{Float64}(I(3)), 
            1, 1, 11, n_timesteps), # fake for now
        joint_start,
        joint_target,
        nom_traj,
        end_goal
    )
    
    # compute u0 from nom_traj
    u0 = diff(nom_traj, dims=2)/PLAN_DT;
    model = ManipVelCtrl{7}(0);
    weights = SVector{6}(req.weights);
    
    comoto_prob = get_partial_comoto(model, subprob_info, weights, u0);
    solver = ALTROSolver(comoto_prob, opts);
    solve!(solver);

    t_finish = time();
    t_elapsed = t_finish - t_start;
    t_covered = float(req.pred_traj.points[1].time_from_start);
    out_traj = JointTrajectory();
    for state_vec in TO.states(solver)
        if t_covered > t_elapsed
            push!(out_traj.points, JointTrajectoryPoint());
            out_traj.points[end].positions = state_vec;
            out_traj.points[end].time_from_start = Duration(t_covered - t_elapsed);
        end
        t_covered += PLAN_DT;
    end
    RobotOS.logwarn("Solved $t_elapsed sec");
    ret_val = SolveResponse();
    ret_val.solve_traj = out_traj;
    return ret_val;
end

function main()
    model = ManipVelCtrl{7}(0)

    # set up a toy problem such that we can call get_probinfo...
    joint_start = @SVector [2.2410335457029205, 2.816951186180262, 3.5541107138047776, 1.3378698662699502, -0.207085307, 4.022068863696378, 5.031459114793775];
    joint_target = @SVector [0.9787559756572425, 2.196178189778825, 2.9467991977653645, 1.9758835017782879, 0.12117641785828975, 4.068015316719079, 5.032986703995045];
    n_timesteps = 20;

    jaco_joints = ["HalfArm1_Link", "HalfArm2_Link", "ForeArm_Link", "SphericalWrist1_Link", "SphericalWrist2_Link", "Bracelet_Link", "EndEffector_Link"];
        
    jaco_info = RobotInfo(
        jaco_joints,
        "novis-jaco_gen3.urdf",
        "EndEffector_Link",
        7
    )
    # jaco_joints = ["j2s7s300_link_1", "j2s7s300_link_2", "j2s7s300_link_3", "j2s7s300_link_4", 
    #     "j2s7s300_link_5", "j2s7s300_link_6", "j2s7s300_link_7"];
        
    # jaco_info = RobotInfo(
    #     jaco_joints,
    #     "novis-jaco.urdf",
    #     "j2s7s300_ee_base",
    #     7
    # )
    object_set = SArray{Tuple{3,2}}(reshape([0.752,-0.19,0.089, 0.752, 0.09, -0.089], (3,2)));
    params = ComotoParameters(
        joint_start,
        joint_target,
        n_timesteps,
        PLAN_DT,
        object_set,
        joint_target
    )
    base_prob_info = get_probinfo(params, jaco_info, "human_trajs/669-means-fmt.csv", 
        "human_trajs/669-vars-fmt.csv")

    opts = SolverOptions(
        penalty_scaling=10.,
        active_set_tolerance_pn=0.01,
        iterations_inner=45,
        iterations_outer=8,
        penalty_initial=150.,
        verbose=0,
        bp_reg=true,
        bp_reg_initial=0.01, # 3500.0, # 0.01,
        cost_tolerance_intermediate=1e-1,
        # projected_newton_tolerance=1e-2,
        # ρ_chol=0.1,
        constraint_tolerance=1e-2
    )

    # trigger precompile by running replan early
    req = SolveRequest();

    # pred_traj, nom_traj
    for i=1:3
        push!(req.pred_traj.points, JointTrajectoryPoint());
        req.pred_traj.points[end].positions = rand(33);

        push!(req.nom_traj.points, JointTrajectoryPoint());
        req.nom_traj.points[end].positions = rand(7);
    end

    # joint_start, start_vel, obj_set, weights
    req.joint_start = rand(7);
    req.start_vel = rand(7);
    req.object_set = rand(6);
    req.weights = rand(6);
    req.end_goal = rand(7);
    
    for i=1:20
        replan(req, jaco_info, base_prob_info, opts);
    end
    
    function do_replan(x::SolveRequest)
        return replan(x, jaco_info, base_prob_info, opts);
    end
    
    pred_sub = Service{Solve}("/comoto", do_replan)
    println("Subscribed!");
    spin();
end

function get_partial_comoto(model::TO.AbstractModel, info::ComotoProblemInfo, weights::AbstractVector, U0::AbstractMatrix)
    final_costs = get_sliding_costs(info, weights);
    n_timesteps = info.n_timesteps;
    tf = (n_timesteps-1)*info.dt;
    obj = TO.Objective(final_costs);

    cons = TO.ConstraintList(info.ctrl_dims, info.state_dims, n_timesteps);
    add_constraint!(cons, TO.BoundConstraint(info.ctrl_dims, info.state_dims, u_min=-0.3, u_max=0.3), 1:n_timesteps-1);
    # cannot constrain final timestep twice
    add_constraint!(cons, TO.BoundConstraint(info.ctrl_dims, info.state_dims, x_min=-2π, x_max=2π), 1:n_timesteps-1);
    if n_timesteps > 2
        add_constraint!(cons, PosEECons(info.ctrl_dims, info.ctrl_dims, SA_F64[-100, -100, 0.4], SA_F64[100,0.02,100], info.full_fk), 2:n_timesteps-1);
    end
    
    prob = TO.Problem(model, obj, info.joint_start, tf, xf=info.joint_target, constraints=cons);
    initial_controls!(prob, U0);
    prob
end

init_node("comoto", anonymous=true);
main()
