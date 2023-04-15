#!/usr/bin/env python3

import rospy
import torch
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from predictor_pkg.training.experiment import model_factory
import predictor_pkg.utils.utils as utils   

import time
from math import floor
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

PUB_PD = 0.25 # predict 4x/s

class PoseListener:
    def __init__(self, pred_pub):
        # prepare predictor & pose queue
        self.model = model_factory()
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

    def resample_traj(self):
        """
        Discard unneccessary traj points and return a vectorized, resampled trajectory
            appropriate to POTR's required sampling rate 

        Returns either a (50, 21*3) resampled trajectory or None (if not enough history)
        """
        self.traj_mut.acquire()
        if len(self.traj_times) < 2 or (self.traj_times[-1] - self.traj_times[0]) < 2.0:
            # not enough collected history, return
            self.traj_mut.release()
            return None

        traj = np.asarray(self.traj_posns) # (n_sample, 21*3)
        traj_times = np.asarray(self.traj_times)
        self.traj_mut.release()

        x_out = np.linspace(traj_times[-1] - 2.0, traj_times[-1], 50) # 2 sec * 25 Hz = 50

        ret_val = np.empty((50, 21*3))
        for i in range(21*3):
            ret_val[:,i] = np.interp(x_out, traj_times, traj[:,i])    

        return ret_val
    
    def predict_cb(self, evt):
        """
        Timer callback
        Publish a prediction given the current user trajectory 
        """
        points = self.resample_traj()
        if points is None:
            return
            
        points = points.reshape(points.shape[0], _JOINT_NUM, 3)
        # use helper function provided in POTR for conversion expmap(dim=3) -> rotmat(dim=9)
        rotmat_points = utils.expmap_to_rotmat(points)
        rotmat_tensor = torch.Tensor(np.asarray(rotmat_points))

        traj = self.predict(rotmat_tensor)
        self.pred_pub.publish(traj)

    '''
        Model inputs:
        1. encoder_input of size (1, frame_num-1, _JOINT_NUM*dim)
            Note: last observation is discarded
        2. decoder_input(query) of size (1, prediction_frame, _JOINT_NUM*dim)
            Note: this is created by duplicating the last observation
    
        input_sequence is a tensor of dim (n_ts, n_joint, 3, 3) (a list of rotation matrices)
    '''
    @torch.no_grad()
    def predict(self, input_sequence):
        encoder_input = input_sequence.view(1, -1, _JOINT_NUM*9)[:, :-1, :].cuda()
        decoder_input = encoder_input[0, -1, :].repeat(1, 25, 1).cuda()
        
        decoder_pred = self.model._model(
            encoder_input, decoder_input)[0][-1]

        print("decoder pred shape: ", decoder_pred.shape)

        pred = decoder_pred.view(decoder_pred.shape[1], _JOINT_NUM, 9).cpu().numpy()
        trajectory_points = utils.rotmat_to_expmap(pred)

        # form JointTrajectory 
        traj_plist = []
        for (i, point) in enumerate(trajectory_points):
            traj_point = JointTrajectoryPoint()
            print(point)
            traj_point.positions = point.flatten()
            
            traj_point.time_from_start = rospy.Duration((i+1)*0.04)
            traj_plist.append(traj_point)
        # Prediction in rotation matrix format 
        pred_trajectory = JointTrajectory()
        pred_trajectory.points = traj_plist
        pred_trajectory.joint_names = _MAJOR_JOINTS
        return pred_trajectory


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
