using RobotDynamics, Rotations
using TrajectoryOptimization
using StaticArrays, LinearAlgebra
using RigidBodyDynamics;
using Altro;
import ForwardDiff;
using Dates;

include("ros_interface.jl")

const RBD = RigidBodyDynamics;
const TO = TrajectoryOptimization;

RobotDynamics.@autodiff struct ManipVelCtrl{n} <: RobotDynamics.ContinuousDynamics
    id::Int32
end

RobotDynamics.control_dim(::ManipVelCtrl{n}) where n = n
RobotDynamics.state_dim(::ManipVelCtrl{n}) where n = n

function RobotDynamics.dynamics(model::ManipVelCtrl, x, u)
    u
end

function RobotDynamics.dynamics!(model::ManipVelCtrl, xdot, x, u)
    xdot .= u;
    return nothing
end

include("ee_posn_cons.jl");
include("costs.jl")

"""
Convenience function to construct comoto costs, add typical constraints for the iiwa,
    set up joint-space linear initial controls, and return the trajopt problem

model: robot model
weights: comoto weights
"""
function get_comoto_problem(model::TO.AbstractModel, info::ComotoProblemInfo, weights::AbstractVector)
    final_costs = get_comoto_costs(info, weights)
    n_timesteps = info.n_timesteps
    tf = (n_timesteps-1)*info.dt;
    obj = TO.Objective(final_costs);
    ctrl_linear = (info.joint_target - info.joint_start)/tf;
    U0 = [ctrl_linear for _ in 1:(n_timesteps-1)];

    cons = TO.ConstraintList(info.ctrl_dims, info.state_dims, n_timesteps);
    add_constraint!(cons, TO.GoalConstraint(info.joint_target), n_timesteps);
    add_constraint!(cons, TO.BoundConstraint(info.ctrl_dims, info.state_dims, u_min=-10, u_max=10), 1:n_timesteps-1);
    # cannot constrain final timestep twice
    add_constraint!(cons, TO.BoundConstraint(info.ctrl_dims, info.state_dims, x_min=-2π, x_max=2π), 1:n_timesteps-1);
    add_constraint!(cons, PosEECons(info.ctrl_dims, info.ctrl_dims, SA_F64[-100, -100, 0], SA_F64[100,0.1,100], info.full_fk), 2:n_timesteps-1);

    prob = TO.Problem(model, obj, info.joint_start, tf, xf=info.joint_target, constraints=cons);
    initial_controls!(prob, U0);
    prob
end

"""
Convenience function to confirm start, move to trajectory start,
    confirm execution, and execute a trajectory through ROS
"""
function confirm_display_traj(solver::ALTROSolver, total_time::Float64, human_trajfile::String="")
    # TODO: change this and exec_human_traj.py to use total time (to avoid different dt's)
    println("Ready to move to start?")
    readline(stdin)
    move_to(TO.states(solver)[1], 4.0)
    println("Ready to dispatch?")
    readline(stdin)
    @sync begin
        human_dt = total_time/(countlines(human_trajfile)-1);
        if human_trajfile != ""
            @async dispatch_human_trajectory(human_trajfile, human_dt);
        end
        robot_dt = total_time/(length(TO.states(solver))-1);
        @async dispatch_trajectory(hcat(TO.states(solver)...), robot_dt, 0.);
    end
end
