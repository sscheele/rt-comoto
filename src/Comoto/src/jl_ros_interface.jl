"""
Construct a JointTrajectory from traj and dispatch on robot's control publisher 

traj should have shape (n_dof, n_points)
"""
function dispatch_trajectory(traj::AbstractMatrix, info::RobotInfo, dt::Float64, t_0::Float64=0.)
    n_points = traj.size[2];
    pt_arr = Vector{JointTrajectoryPoint}(undef, n_points);
    for i=1:n_points
        tmp = JointTrajectoryPoint();
        tmp.positions = traj[:,i];
        tmp.time_from_start = Duration((i-1)*dt + t_0);
        pt_arr[i] = tmp;
    end
    msg = JointTrajectory()
    msg.points = pt_arr;
    msg.joint_names = info.jnames;
    publish(info.ctrl_pub, msg);
end


