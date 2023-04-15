#!/usr/bin/env python3
import rospy
import numpy as np
import pickle
import sys

from time import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PredRecorder:
    def __init__(self, max_steps=10):
        self.max_steps = max_steps
        self.n_steps = 0
        
        self.t0 = None
        self.trajs = []
        self.time_offsets = []
        
    def traj_cb(self, traj: JointTrajectory):
        self.n_steps += 1
        print("mooooooooooooo")
        if self.n_steps > self.max_steps:
            rospy.signal_shutdown("done")
            sys.exit()
            
        self.trajs.append(traj)
        now = time()
        if self.t0 is None:
            self.t0 = now
        self.time_offsets.append(now - self.t0)
        
        # if ready, dump to pickle
        if self.n_steps == self.max_steps:
            with open('traj.pkl', 'wb') as f:
                pickle.dump((self.trajs, self.time_offsets), f)

if __name__ == "__main__":
    rospy.init_node("pred_recorder", anonymous=True)
    if len(sys.argv) == 2 and sys.argv[1] == "replay":
        with open('traj.pkl', 'rb') as f:
            traj_list, offsets = pickle.load(f)
        # traj_pub = rospy.Publisher('/time_traj', JointTrajectory, queue_size=1)
        traj_pub = rospy.Publisher('/prediction_traj', JointTrajectory, queue_size=1)
        traj_pub.publish(traj_list[0])
        offsets = np.diff(offsets)
        traj_pub.publish(traj_list[0])
        for offs_idx, offs in enumerate(offsets):
            rospy.sleep(offs)
            curr_time = time()
            traj = traj_list[offs_idx+1]
            traj_pub.publish(traj)
    else:
        tmp = PredRecorder()
        rospy.sleep(5)
        print("************\nON\n**************")
        rospy.Subscriber("/prediction_traj", JointTrajectory, tmp.traj_cb, queue_size=1)
        rospy.spin()
