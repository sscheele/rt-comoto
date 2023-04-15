using RigidBodyDynamics;
using LinearAlgebra;
using RobotOS;
import PyCall;

include("util.jl")
include("comoto_structs.jl")
include("comoto_base.jl")

function get_joint(robot::Mechanism, jointname::String)
    ee_body = findbody(robot, jointname)
    ee_point = Point3D(default_frame(ee_body),0.,0.,0.)
    return ee_body, ee_point
end

function get_full_fk_fn(info::RobotInfo, robot::Mechanism, cache::StateCache)
    function full_fk(x::AbstractVector{T}) where T<:Real
        state_vec = T[];
        state = cache[T];
        world = root_frame(robot);
        nn = num_positions(robot);

        # RBD.set_configuration!(state, x[1:nn])
        RBD.set_configuration!(state, x)
        idx = 1;
        for jointname in info.jnames
            body, point = get_joint(robot, jointname);
            append!(state_vec, RBD.transform(state, point, world).v);
            idx += 3;
        end
        reshape(state_vec, (3,:))
    end
end

function get_eef_trans_fn(info::RobotInfo, robot::Mechanism, cache::StateCache)
    ee_body, ee_point = get_joint(robot, info.eef_name);

    world = root_frame(robot)
    nn = num_positions(robot)

    function ee_position(x::AbstractVector{T}) where T
        state = cache[T]
        RBD.set_configuration!(state, x[1:nn])
        # RBD.transform(state, ee_point, world)
        RBD.transform_to_root(state, ee_body)
    end
end

function get_probinfo(params::ComotoParameters, info::RobotInfo, 
    means_filepath::String="means.csv", vars_filepath::String="vars.csv")
    
    joint_tree = parse_urdf(info.urdf_path, remove_fixed_tree_joints=false)
    # print(joint_tree)
    cache = StateCache(joint_tree);

    end_effector_fn = get_eef_trans_fn(info, joint_tree, cache);
    full_fk_fn = get_full_fk_fn(info, joint_tree, cache);
    
    # TODO: BEWARE OF HARDCODED OFFSET YIKES YIKES YIKES
    # human_traj, head_traj, human_vars_traj, human_goal = read_human_traj_files(means_filepath, vars_filepath, offset=[0.5, 0., -0.75]);
    human_traj, head_traj, human_vars_traj, human_goal = read_human_traj_files(means_filepath, vars_filepath);
    human_traj = resample_human_traj(human_traj, params.n_timesteps);
    head_traj = resample_human_traj(head_traj, params.n_timesteps);
    human_vars_traj = resample_human_traj(human_vars_traj, params.n_timesteps);

    nom_traj = get_nominal_traj(params.joint_start, params.joint_target, params.n_timesteps);

    ComotoProblemInfo(
        joint_tree,
        end_effector_fn,
        full_fk_fn,
        info.dof,
        info.dof,
        params.n_timesteps,
        params.dt,
        params.goal_set[:,1],
        params.goal_set,
        human_goal,
        human_traj,
        head_traj,
        human_vars_traj,
        params.joint_start,
        params.joint_target,
        nom_traj,
        params.end_goal
    )
end
