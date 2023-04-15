import Pkg;
Pkg.activate("/home/sscheele3/Comoto")

using RobotOS;

@rosimport trajectory_msgs.msg: JointTrajectory, JointTrajectoryPoint
rostypegen();
using .trajectory_msgs.msg;

float(x::Duration) = x.secs + (x.nsecs/1e9);

function print_earliest(traj::JointTrajectory)
    println("Hello");
    traj_t0 = float(traj.points[1].time_from_start);
    now = time();
    println("Earliest joint traj time is ", traj_t0, " but current time is ", now);
    println("Difference of: ", traj_t0 - now);
end

init_node("time_test", anonymous=true);
print_sub = Subscriber{JointTrajectory}("/time_traj", print_earliest, queue_size=1);
println("Ready!")
spin();
