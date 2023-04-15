"ComotoProblemInfo contains all neccessary information to compute all comoto costs"
struct ComotoProblemInfo
    joint_tree::RigidBodyDynamics.Mechanism
    eef_fk::Function
    full_fk::Function
    ctrl_dims::Int
    state_dims::Int
    n_timesteps::Int
    dt::Float64
    object_pos::AbstractVector
    goal_set::AbstractMatrix
    human_goal::AbstractVector
    human_traj::AbstractArray
    head_traj::AbstractArray
    human_vars_traj::AbstractArray
    joint_start::AbstractVector
    joint_target::AbstractVector
    nominal_traj::AbstractMatrix
    # the end goal is not necessarily the target of this rollout, since we're doing MPC
    end_goal::AbstractVector
end

"RobotInfo contains a robot specification, including joint names, urdf path,
end effector name, degrees of freedom, and name of controller rostopic"
struct RobotInfo
    jnames::Vector{String};
    urdf_path::String;
    eef_name::String;
    dof::Int;
    # ctrl_pub::Publisher{JointTrajectory};
end

"ComotoParameters are the user-defined paramters for a comoto problem"
mutable struct ComotoParameters
    joint_start::AbstractVector
    joint_target::AbstractVector
    n_timesteps::Int
    dt::Float64
    goal_set::AbstractMatrix
    end_goal::AbstractVector
end
