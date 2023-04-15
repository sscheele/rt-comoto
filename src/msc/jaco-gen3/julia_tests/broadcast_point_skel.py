"""
    Broadcasts a "human skeleton trajectory" where all joints are in the same
    place on /prediction_traj every 0.5 sec. Trajectory will be 25 Hz and
    times from start will be real times.
"""
import rospy
import numpy as np
from time import time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == "__main__":
    rospy.init_node("point_skel", anonymous=True)
    pub = rospy.Publisher("/prediction_traj", JointTrajectory, queue_size=1)
    
    while True:
        msg = JointTrajectory()
        t_0 = time()
        for i in range(25):
            point = JointTrajectoryPoint()
            x = np.zeros(21*3)
            x[1::3] = -0.6
            x[2::2] = 0.32
            point.positions = list(x)
            point.time_from_start = rospy.Duration(t_0 + 0.04*i)
            msg.points.append(point)
        pub.publish(msg)
        rospy.sleep(0.5)
