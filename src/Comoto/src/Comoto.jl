module Comoto

using RobotOS;
import FiniteDiff;

@rosimport trajectory_msgs.msg: JointTrajectory, JointTrajectoryPoint
@rosimport julia_comoto.srv: Solve
rostypegen(@__MODULE__)
using .trajectory_msgs.msg;

include("problem_info.jl");

OBJECT_SET = SArray{Tuple{3,2}}(reshape([0.752,-0.19,0.089, 0.752, 0.09, -0.089], (3,2)));
OBJECT_POS = OBJECT_SET[:,1];

function get_partial_comoto(model::TO.AbstractModel, info::ComotoProblemInfo, 
        weights::AbstractVector, U0::AbstractVector)
    
    final_costs = get_sliding_costs(info, weights)
    n_timesteps = info.n_timesteps
    tf = (n_timesteps-1)*info.dt;
    obj = TO.Objective(final_costs);

    cons = TO.ConstraintList(info.ctrl_dims, info.state_dims, n_timesteps);
    add_constraint!(cons, TO.BoundConstraint(info.ctrl_dims, info.state_dims, u_min=-10, u_max=10), 1:n_timesteps-1);
    # cannot constrain final timestep twice
    add_constraint!(cons, TO.BoundConstraint(info.ctrl_dims, info.state_dims, x_min=-2π, x_max=2π), 1:n_timesteps-1);
    if n_timesteps > 2
        add_constraint!(cons, PosEECons(info.ctrl_dims, info.ctrl_dims, SA_F64[-100, -100, 0], SA_F64[100,0.1,100], info.full_fk), 2:n_timesteps-1);
    end
    
    prob = TO.Problem(model, obj, info.joint_start, tf, xf=info.joint_target, constraints=cons);
    initial_controls!(prob, U0);
    prob
end

function main()
    model = ManipVelCtrl{7}(0)

    joint_start = @SVector [2.2410335457029205, 2.816951186180262, 3.5541107138047776, 1.3378698662699502, 6.076088313435101, 4.022068863696378, 5.031459114793775];
    joint_target = @SVector [0.9787559756572425, 2.196178189778825, 2.9467991977653645, 1.9758835017782879, 0.12117641785828975, 4.068015316719079, 5.032986703995045];
    n_timesteps = 20;

    jaco_joints = ["j2s7s300_link_1", "j2s7s300_link_2", "j2s7s300_link_3", "j2s7s300_link_4", "j2s7s300_link_5", "j2s7s300_link_6", "j2s7s300_link_7"];
        
    jaco_info = RobotInfo(
        jaco_joints,
        "novis-jaco.urdf",
        "j2s7s300_ee_link",
        7
    )
    params = ComotoParameters(
        joint_start,
        joint_target,
        n_timesteps,
        0.25,
        OBJECT_SET
    )

    base_prob_info = get_probinfo(params, jaco_info, "human_trajs/669-means-fmt.csv", 
        "human_trajs/669-vars-fmt.csv")
    
    weights = @SVector [2., 1.5, 24.8, 1., 1., 10.];
    u0 = (joint_start - joint_target)./(n_timesteps-1);
    prob = get_partial_comoto(model, base_prob_info, weights, u0);

    println("Beginning to attempt solution");

    opts = SolverOptions(
        penalty_scaling=10.,
        active_set_tolerance_pn=0.01,
        iterations_inner=60,
        iterations_outer=5,
        penalty_initial=0.01,
        verbose=0,
        bp_reg=true,
        bp_reg_initial=0.01,
        cost_tolerance_intermediate=1e-1,
        # projected_newton_tolerance=1e-2,
        # ρ_chol=0.1,
        constraint_tolerance=1e-2
    )

    solver = ALTROSolver(prob, opts);
    solve!(solver)
    println("Cost: ", cost(solver))
    println("States: ", TO.states(solver))
    println("Controls: ", TO.controls(solver))
    println("Violated joint constraints: ", any(x->any(y->y<-2π||y>2π, x), TO.states(solver)))
    println("Violated control constraints: ", any(x->any(y->y<-5||y>5, x), TO.controls(solver)))
    println("Reaches goal: ", sq_norm(joint_target - TO.states(solver)[end]) < 0.01)
end

precompile(main, ());
end
