import numpy as np
import rospy
import sys

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal 

JOINT_TARGET = [-1.3055470801196583, -0.9100197213432706, 0.3673008472777747, -0.8728350683069293, -0.31091180672573593, -1.2278695953408816, 1.4791600163205234]


# give 2 seconds to reach start
traj_states = [traj_states[0]] + traj_states
traj_times = np.array(traj_times) - traj_times[0] 
traj_times = np.concatenate(([0], traj_times + 2))

def go_to_point(pub, point):
    out_goal = FollowJointTrajectoryActionGoal()    
    out_traj = JointTrajectory()
    # TODO use self.joint_names
    # out_traj.joint_names = [f"Actuator{i}" for i in range(1,8)]
    out_traj.joint_names = [f"joint_{i}" for i in range(1,8)]
    traj_point = JointTrajectoryPoint()
    traj_point.time_from_start = rospy.Duration(secs=3)
    traj_point.positions = list(point)
    out_traj.points.append(traj_point)
    out_goal.goal.trajectory = out_traj
    rospy.sleep(1)
    pub.publish(out_goal)

if __name__ == "__main__":
    rospy.init_node("do_traj", anonymous=True)
    traj_pub = rospy.Publisher("/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=1)
    
    input("Enter to move to start...")
    go_to_point(traj_pub, traj_states[0])
    input("Enter to start moving...")
    
    if len(sys.argv) > 1 and sys.argv[1] == 'nom':
        go_to_point(traj_pub, JOINT_TARGET)
    else:
        out_goal = FollowJointTrajectoryActionGoal()    
        out_traj = JointTrajectory()
        # TODO use self.joint_names
        # out_traj.joint_names = [f"Actuator{i}" for i in range(1,8)]
        out_traj.joint_names = [f"joint_{i}" for i in range(1,8)]
        for idx, pt in enumerate(traj_states):
            traj_point = JointTrajectoryPoint()
            traj_point.time_from_start = rospy.Duration(nsecs=traj_times[idx])
            traj_point.positions = list(pt)
            out_traj.points.append(traj_point)
        out_goal.goal.trajectory = out_traj
        rospy.sleep(1)
        traj_pub.publish(out_goal)
