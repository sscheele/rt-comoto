#!/usr/bin/env python3

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import time
from threading import Lock
'''
<Pose prediction script>
Listen to topic 'human_posn' with type JointTrajectoryPoint

Accumulate a 2-sec pose sequence, reformat, then predict a trajectory of 25 frame (1 sec)

Publish to topic 'prediction_traj' in type JointTrajectory(
    header: empty
    joint_names: order of indices as in H36M,
    points: frames of poses of size (frame_num, joint_num=21, exp_coordinate_dim=3)
)
'''
_MAJOR_JOINTS = [
    '0', '1', '2', '3', '4', '6', '7', '8', '9', '11', '12', '13', '14', '16', '17', '18', '19', '24', '25', '26', '27'
]
_JOINT_NUM = 21

PUB_PD = 0.4 # predict 2x/s

class PoseListener:
    def __init__(self, pred_pub):
        # prepare predictor & pose queue
        self.pred_pub = pred_pub

        self.traj_posns = []
        self.traj_times = []
        self.traj_mut = Lock()        

    def posn_cb(self, pose: JointTrajectoryPoint):
        self.traj_mut.acquire()
        now = time.time()
        self.traj_times.append(now)
        self.traj_posns.append(pose.positions)
        
        # discard old points to prevent memory waste
        if now - self.traj_times[0] > 2.0:
            first_idx = 0
            while now - self.traj_times[first_idx] >= 2.0:
                first_idx += 1
            first_idx -= 1
            self.traj_posns = self.traj_posns[first_idx:]
            self.traj_times = self.traj_times[first_idx:]
        self.traj_mut.release()

    def predict_cb(self, evt):
        """
        Timer callback
        Publish a prediction given the current user trajectory 
        """
        self.traj_mut.acquire()
        traj = np.array(self.traj_posns)
        times = np.array(self.traj_times)
        self.traj_mut.release()
        if len(times) < 2:
            return

        # times is up to 2s
        times = times - times[0]
        
        n_out_pts = 50 # 2 sec at 25 Hz
        out_arr = np.empty((n_out_pts, 21*3))
        # x_out is a column vector of times beyond 2s
        x_out = 2 + np.reshape((1 + np.arange(n_out_pts))/25, (-1, 1)) 

        # doing a parabolic fit, so x_infer should be (n_out_pts, 3)
        degree = 1
        x_infer = np.concatenate(tuple(
            np.power(x_out, degree - i) for i in range(degree + 1)
        ), axis=1)
        for i in range(21*3):
            # fit on t in [0, 2]
            coeffs = np.polyfit(times, traj[:, i], degree).reshape((-1, 1)) # (3, 1)
            # predict on t in (2, n_out_pts/25)
            pred = np.matmul(x_infer, coeffs) # (n_out_pts, 1)
            out_arr[:,i] = pred.ravel()

        traj = JointTrajectory()
        for i in range(n_out_pts):
            pt = JointTrajectoryPoint()
            # -2 because we added 2 to make times positive during inference
            pt.time_from_start = rospy.Duration(x_out[i] - 2)
            pt.positions = out_arr[i,:]
            traj.points.append(pt)
        self.pred_pub.publish(traj)
            


if __name__ == '__main__':
    # allow some time for ros initialization
    time.sleep(1) 
    # prepare node
    rospy.init_node('traj_listener', anonymous=True)
    pred_pub = rospy.Publisher('prediction_traj', JointTrajectory, queue_size=1)
    listener = PoseListener(pred_pub)
    rospy.Subscriber('human_posn', JointTrajectoryPoint, listener.posn_cb, queue_size=1)
    rospy.Timer(rospy.Duration(PUB_PD), listener.predict_cb)
    rospy.spin()
