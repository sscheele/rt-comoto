#!/usr/bin/env python
from random import sample
import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


'''
Generate raw trajectories from H36M as test data
Each trajectory published will include 5 frames i.e. total 5*21*3 entries

'''
_MAJOR_JOINTS = [
    '0', '1', '2', '3', '4', '6', '7', '8', '9', '11', '12', '13', '14', '16', '17', '18', '19', '24', '25', '26', '27'
]

# Generate random trajectory 
def random_traj_gen(frame):
    traj_list = []
    
    for j in range(frame):
        pose = JointTrajectoryPoint()
        pose.positions = np.random.uniform(0, 1, 63)
        traj_list.append(pose)

    traj = JointTrajectory()
    traj.joint_names = _MAJOR_JOINTS
    traj.points = traj_list
    return traj

def talker():
    traj_pub = rospy.Publisher('human_traj', JointTrajectory, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # hz
    while not rospy.is_shutdown():
        # Manually generate new msgs for testing
        sample_traj = random_traj_gen(25)
        traj_pub.publish(sample_traj)
        rate.sleep()

if __name__ == '__main__':
    
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
