#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal

traj = [
	[2.455903241404912, 2.6802668400769263, 3.6845299734437025, 1.6768776230218594, 4.442554569260662, 4.487989961670177, 5.031454321103813],
	[2.4159355850294384, 1.8691232326314093, 3.6645522705264733, 1.6867010921767802, 3.963883054935891, 4.772793734965431, 5.029897437130552],
	[0.7729679255885306, 2.0418784978032143, 2.7550414792675166, 1.3531655988327658, 2.2656355615369637, 4.762985845302888, 5.029912350832657],
	[0.6979773040335344, 2.8576291076184597, 2.7550380171580997, 1.332752602235363, 1.8438361186069434, 4.437606149639186, 5.029909687671567]
]

def make_traj(arr, time_per_point=2, base_time=0):
    msg = JointTrajectory()
    for i, item in enumerate(arr):
        tmp = JointTrajectoryPoint()
        tmp.positions = item
        tmp.time_from_start = rospy.Duration(base_time + i*time_per_point)
        msg.points.append(tmp)
    return msg, tmp.time_from_start.to_sec()

if __name__ == "__main__":
    rospy.init_node("traj_test", anonymous=True)
    pub = rospy.Publisher("/jaco_adaptive_ctrl/goal", JointTrajectory, queue_size=1)
    rospy.sleep(1)
    
    msg, t = make_traj(traj[:1], base_time=4)
    pub.publish(msg)
    rospy.sleep(1.0)
    msg, t = make_traj(traj[1:3], base_time=t+3)
    pub.publish(msg)
    rospy.sleep(1.0)
    msg, t = make_traj(traj[3:], base_time=t+2)
    pub.publish(msg)