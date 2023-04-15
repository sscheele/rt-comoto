#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from baselines import positions

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
    
    goal = positions.posns[1,:]
    # goal = [1.0656882781191814, 0.4604144797878419, -3.118373350993719, -1.2922323399336983, -0.1051192292831713, -1.3377377734798825, 1.806642277664842]
    msg, _ = make_traj([goal], base_time=6)
    pub.publish(msg)
    rospy.sleep(0.1)    
