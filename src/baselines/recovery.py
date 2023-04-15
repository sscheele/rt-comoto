#!/usr/bin/env python3
import rospy
import numpy as np

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from baselines.nominal import cart2jnt, jaco_ee_fk

def do_recovery():
    curr_posn = rospy.wait_for_message(
        '/right_arm/base_feedback/joint_state', JointState)
    curr_posn = np.array(curr_posn.position[:7])
    curr_cart_posn = jaco_ee_fk(curr_posn)
    goal_cart_posn = curr_cart_posn.copy()
    goal_cart_posn[1] -= 0.05
    goal_cart_posn[2] += 0.05

    traj_pub = rospy.Publisher('/jaco_adaptive_ctrl/goal', 
        JointTrajectory, queue_size=None, tcp_nodelay=True)
        
    cart_traj = np.array([curr_cart_posn, goal_cart_posn])
    traj, times = cart2jnt(cart_traj, curr_posn, curr_posn, cart_tol = 0.01)
    traj = traj[:-1]
    times = times[:-1]
    times = 2.7*times
    
    msg = JointTrajectory()
    for i in range(traj.shape[0]):
        pt = JointTrajectoryPoint()
        pt.time_from_start = rospy.Duration(times[i])
        pt.positions = list(traj[i,:])
        msg.points.append(pt)
    
    rospy.sleep(0.1)
    traj_pub.publish(msg)
    traj_dur = msg.points[-1].time_from_start.to_sec()
    print("Traj lasts %f sec"%traj_dur)
    rospy.sleep(0.5)

if __name__ == "__main__":
    rospy.init_node("rrt_recovery", anonymous=True)
    do_recovery()
