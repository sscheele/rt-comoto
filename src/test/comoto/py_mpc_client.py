#!/usr/bin/env python3
import numpy as np
import rospy

from threading import Lock
from scipy.interpolate import CubicSpline
from math import ceil
from time import time

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from julia_comoto.msg import SolveRequest

PRED_DT = 0.04 # prediction sampling period/timestep
PLAN_DT = 0.25
N_HUMAN_JOINTS = 11
PLAN_TIME = 0.1 # time to plan in sec

MAX_U_MAG = 0.2
TRAJ_DURATION = 10
# LEG, VIS, DIST, NOM, VEL, POSE
# WEIGHTS = [2., 1.5, 1.4, 1., 1., 10.]
WEIGHTS = [0., 0, 0, 2., 0., 0.]
GOAL_THRESH = 0.1

class TrajectoryManager:
    def __init__(self, goal, weights, req_pub):
        self.goal = np.asarray(goal)
        self.weights = weights
        self.req_pub = req_pub
        
        self.start_time = time()

        self.joint_state_mut = Lock()
        self.curr_posn = None
        self.curr_vel = None

    def joint_state_cb(self, s: JointState):
        """
        Save the most recent joint position and velocity 
        """
        self.joint_state_mut.acquire()
        self.curr_posn = np.array(s.position)[:7]
        self.curr_vel = np.array(s.velocity)[:7]
        self.joint_state_mut.release()

    def joint_pred_cb(self, traj: JointTrajectory):
        """
        - Resample traj to planning rate  
        - Change skeleton morphology to be CoMOTO-compatible
        - Construct SolveRequest and send to Julia
        """
        t_remaining = TRAJ_DURATION - (time() - self.start_time)
        if t_remaining <= 0:
            return
            
        self.joint_state_mut.acquire()
        curr_posn = self.curr_posn
        curr_vel = self.curr_vel
        self.joint_state_mut.release()

        if curr_posn is None:
            return
            
        if np.linalg.norm(self.goal - curr_posn) < GOAL_THRESH:
            print("Reached goal!!")
            return
        
        traj_matr = np.array(
            [x.positions for x in traj.points]
        )
        # remove timesteps which can't be solved within PLAN_TIME
        n_ts_removed = ceil(PLAN_TIME/PRED_DT)
        if n_ts_removed > traj_matr.shape[0]:
            print("ERROR: can't plan a {(PRED_DT-1)*traj_matr.shape[0]} sec " + 
                "trajectory in {PLAN_TIME} sec")
            return
        
        traj_matr = traj_matr[n_ts_removed:,:]
        n_joints = traj_matr.shape[1]
        n_plan_ts = 1 + int(
            ((len(traj.points)-1)*PRED_DT)//PLAN_DT
        )

        orig_pts = PRED_DT*np.arange(traj_matr.shape[0])
        eval_pts = PLAN_DT*np.arange(n_plan_ts)

        resampled = np.zeros((n_plan_ts, n_joints))
        for i in range(n_joints):
            resampled[:,i] = np.interp(eval_pts, orig_pts, traj_matr[:,i])

        plan_hum_traj = np.zeros((n_plan_ts, 3*N_HUMAN_JOINTS))
        for i in range(n_plan_ts):
            plan_hum_traj[i,:] = adapt_morphology(resampled[i,:])

        solve_req = SolveRequest()
        
        # add human traj
        for i in range(n_plan_ts):
            pt = JointTrajectoryPoint()
            pt.time_from_start = rospy.Duration(i*PLAN_DT)
            pt.positions = plan_hum_traj[i,:]
            solve_req.pred_traj.points.append(pt)
                
        # add start state (posn and vel)
        # TODO we could do better than this
        solve_req.joint_start = curr_posn + PLAN_TIME*curr_vel
        solve_req.start_vel = curr_vel

        # add weights
        solve_req.weights = self.weights

        # add object set
        # TODO hardcode me to something else!
        solve_req.object_set = [0.752,-0.19,0.089, 0.752, 0.09, -0.089]

        # find nominal traj
        u_nom = 0.5*(self.goal - curr_posn)
        u_nom = np.clip(u_nom, -MAX_U_MAG, MAX_U_MAG)
        nom_target = ((n_plan_ts - 1)*PLAN_DT*u_nom) + curr_posn
        nom_traj = np.linspace(curr_posn, nom_target, n_plan_ts)

        # add nominal traj
        for i in range(n_plan_ts):
            pt = JointTrajectoryPoint()
            pt.time_from_start = rospy.Duration(i*PLAN_DT)
            pt.positions = nom_traj[i,:]
            solve_req.nom_traj.points.append(pt)

        self.req_pub.publish(solve_req)

def adapt_morphology(x):
    """
    x is a vectorized skeleton
    returns a vectorized skeleton with morphology adapted for
        optimization with Comoto
    Input Joints:
        0 - Root (pelvis)
        1 - Right hip
        2 - Right knee
        3 - Right ankle
        4 - Right foot
        5 - left hip
        6 - left knee
        7 - right ankle
        8 - left foot
        9 - duplicate of 0
        10 - center spine
        11 - shoulder spine/base of neck
        12 - bottom of head
        13 - duplicate of 13
        14 - left shoulder
        15 - left elbow
        16 - left wrist
        17 - duplicate of 13
        18 - right shoulder
        19 - right elbow
        20 - right wrist

    Output joints:
        right(shoulder, elbow, wrist, palm), neck, head, torso, left(...)
    """
    output = np.zeros(33)
    
    # right shoulder, elbow, wrist
    output[0:3] = x[18*3:19*3]
    output[3:6] = x[19*3:20*3]
    output[6:9] = x[20*3:21*3]
    # make right palm match wrist for now
    output[9:12] = x[20*3:21*3]

    # neck, head, torso (pelvis)
    output[12:15] = x[11*3:12*3]
    output[15:18] = x[12*3:13*3]
    output[18:21] = x[0:3]

    # left shoulder, elbow, wrist
    output[21:24] = x[14*3:15*3]
    output[24:27] = x[15*3:16*3]
    output[27:30] = x[16*3:17*3]
    # make left palm match left wrist for now
    output[30:33] = x[16*3:17*3]
    return output

if __name__ == "__main__":
    # move to start posn
    # wait for input, then start TrajectoryManager
    rospy.init_node("traj_manager", anonymous=True)
    goal = [1.0656882781191814, 0.4604144797878419, -3.118373350993719, -1.2922323399336983, -0.1051192292831713, -1.3377377734798825, 1.806642277664842]
    comoto_pub = rospy.Publisher("/comoto", SolveRequest, queue_size=1)
    manager = TrajectoryManager(goal, WEIGHTS, comoto_pub)
    rospy.Subscriber("/right_arm/base_feedback/joint_state", JointState, manager.joint_state_cb)
    rospy.Subscriber("/prediction_traj", JointTrajectory, manager.joint_pred_cb)
    rospy.spin()