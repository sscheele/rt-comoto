#!/usr/bin/env python3
import numpy as np
import rospy
import kinpy as kp

from threading import Lock
from scipy.interpolate import CubicSpline
from math import ceil, pi
from time import time

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from julia_comoto.srv import SolveRequest, SolveResponse, Solve

PRED_DT = 0.04 # prediction sampling period/timestep
PLAN_DT = 0.4
N_HUMAN_JOINTS = 11
PLAN_TIME = 0.3 # time to plan in sec

# min distance between user and goal position
USER_GOAL_STANDOFF = 0.15
# maximum angular velocity on any joint
MAX_U_MAG = 0.3
# maximum trajectory duration (sec)
TRAJ_DURATION = 25
# LEG, VIS, DIST, NOM, VEL, POSE
# WEIGHTS = [2., 1.5, 1.4, 1., 1., 10.]
WEIGHTS = [1, 0.6, 0.5, 30., 0., 6.]
GOAL_THRESH = 0.1

def get_posn_nearest(pos, ref):
    """
    Returns the version of our position that is nearest the reference
    Ex: if we are at pi-0.05 and ref is at (0.05-pi), show us at
    (-pi-0.05) 
    """
    # since values are in (-pi, pi), we can simply add/subtract 2*pi
    # iff we're more than pi away from the ref
    out_pos = pos.copy()
    out_pos[ref - out_pos > np.pi] += 2*np.pi
    out_pos[out_pos - ref > np.pi] -= 2*np.pi
    return out_pos
    
def get_nom_traj(goal, start, n_ts):
    curr = start
    ret_val = [curr]
    while len(ret_val) < n_ts:
        u_nom = 0.5*(goal - curr)
        u_nom = np.clip(u_nom, -MAX_U_MAG, MAX_U_MAG)
        curr = curr + PLAN_DT*u_nom
        ret_val.append(curr)
    return np.array(ret_val)    
    
gen3_chain = kp.build_chain_from_urdf(open('novis-jaco_gen3.urdf').read())
gen3_joints = [f'Actuator{i}' for i in range(1,8)]
gen3_eef_name = "EndEffector_Link"

def jaco_ee_fk(posn):
    """
    Posn: length-7 vector
    Returns a 3-length np vector
    """
    fk_map = gen3_chain.forward_kinematics(
        {gen3_joints[i]: posn[i] for i in range(len(posn))})
    return np.array(fk_map[gen3_eef_name].pos)

def human_near_posn(human_posn, posn, thresh=USER_GOAL_STANDOFF):
    """
    Return True if any joint in human_posn is within thresh of posn
    human_posn: (n, 3)
    posn: (3,)
    thresh: float
    """
    return np.any(np.linalg.norm(human_posn - posn, axis=1) < thresh)
    
class TrajectoryManager:
    def __init__(self, goal, weights, solve_srv):
        self.goal = np.asarray(goal)
        self.cart_goal = jaco_ee_fk(self.goal)
        self.weights = weights
        self.solve_srv = solve_srv
        self.traj_pub = rospy.Publisher("/jaco_adaptive_ctrl/goal", 
            JointTrajectory, queue_size=1)
        
        self.start_time = time()
        self.solve_mut = Lock()

        self.state_mut = Lock()
        self.curr_posn = None
        self.curr_vel = None
        self.pred_traj = None
        
        self.reached_goal = False

    def joint_state_cb(self, s: JointState):
        """
        Save the most recent joint position and velocity 
        """
        self.state_mut.acquire()
        self.curr_posn = np.array(s.position)[:7]
        self.curr_vel = np.array(s.velocity)[:7]
        self.state_mut.release()

    def joint_pred_cb(self, traj: JointTrajectory):
        self.state_mut.acquire()
        self.pred_traj = np.array([x.positions for x in traj.points])
        self.state_mut.release()
        
    def mpc_replan(self, evt):
        """
        - Resample traj to planning rate  
        - Change skeleton morphology to be CoMOTO-compatible
        - Construct SolveRequest and send to Julia
        
        Returns: True, if trajectory is complete, False otherwise
        """
        if self.reached_goal:
            return True

        if self.solve_mut.locked():
            return False
        
        t_remaining = TRAJ_DURATION - (time() - self.start_time)
        if t_remaining <= 0:
            return True
            
        self.solve_mut.acquire()
        self.state_mut.acquire()
        if self.curr_posn is None or self.pred_traj is None:
            self.state_mut.release()
            self.solve_mut.release()
            return False

        curr_posn = self.curr_posn.copy()
        curr_vel = self.curr_vel.copy()
        traj_matr = self.pred_traj.copy()
        self.state_mut.release()

        t0_human_posn = traj_matr[0,:].reshape((-1, 3))
        # if human near goal, freeze
        if human_near_posn(t0_human_posn, self.cart_goal):
            pt = JointTrajectoryPoint()
            pt.positions = list(curr_posn)
            pt.time_from_start = rospy.Duration(0)
            msg = JointTrajectory()
            msg.points = [pt]
            self.traj_pub.publish(msg)
            self.solve_mut.release()
            return False
        
        # if we get close to the goal, drive the rest of the way without CoMOTO
        goal = get_posn_nearest(self.goal, curr_posn)
        if np.linalg.norm(self.cart_goal - jaco_ee_fk(curr_posn)) < GOAL_THRESH:
            print("Reached goal!!")
            self.reached_goal = True
            pts = [JointTrajectoryPoint(), JointTrajectoryPoint()]
            pts[0].positions = list(curr_posn)
            pts[0].time_from_start = rospy.Duration(0)
            pts[1].positions = list(goal)
            pts[1].time_from_start = rospy.Duration(6)
            msg = JointTrajectory()
            msg.points = pts
            self.traj_pub.publish(msg)
            self.solve_mut.release()
            return True
        
        # remove timesteps which can't be solved within PLAN_TIME
        n_ts_removed = ceil(PLAN_TIME/PRED_DT)
        if n_ts_removed > traj_matr.shape[0]:
            print("ERROR: can't plan a {(PRED_DT-1)*traj_matr.shape[0]} sec " + 
                "trajectory in {PLAN_TIME} sec")
            self.solve_mut.release()
            return False
        
        traj_matr = traj_matr[n_ts_removed:,:]
        n_pred_ts, n_joints = traj_matr.shape
        n_plan_ts = 1 + int(
            ((n_pred_ts-1)*PRED_DT)//PLAN_DT
        )

        # resample human trajectory to CoMOTO timestep value
        orig_pts = PRED_DT*np.arange(traj_matr.shape[0])
        eval_pts = PLAN_DT*np.arange(n_plan_ts)

        resampled = np.zeros((n_plan_ts, n_joints))
        for i in range(n_joints):
            resampled[:,i] = np.interp(eval_pts, orig_pts, traj_matr[:,i])

        # adapt skeleton morphology to CoMOTO expected morphology
        plan_hum_traj = np.zeros((n_plan_ts, 3*N_HUMAN_JOINTS))
        for i in range(n_plan_ts):
            plan_hum_traj[i,:] = adapt_morphology(resampled[i,:])

        solve_req = SolveRequest()
        
        # add human traj
        for i in range(n_plan_ts):
            pt = JointTrajectoryPoint()
            pt.time_from_start = rospy.Duration(PLAN_TIME + i*PLAN_DT)
            pt.positions = plan_hum_traj[i,:]
            solve_req.pred_traj.points.append(pt)
                
        # add start state (posn and vel)
        # TODO we could do better than this
        solve_req.joint_start = curr_posn + PLAN_TIME*curr_vel
        solve_req.start_vel = curr_vel

        # add weights
        solve_req.weights = self.weights

        # add object set
        solve_req.object_set = [0, -0.55, 0.1, -0.25, -0.55, 0.1, 0.25, -0.55, 0.1]

        # find nominal traj
        nom_traj = get_nom_traj(goal, curr_posn, n_plan_ts)
        
        # add nominal traj
        for i in range(n_plan_ts):
            pt = JointTrajectoryPoint()
            pt.time_from_start = rospy.Duration(i*PLAN_DT)
            pt.positions = nom_traj[i,:]
            solve_req.nom_traj.points.append(pt)

        resp: SolveResponse = self.solve_srv(solve_req)
        
        self.traj_pub.publish(resp.solve_traj)
        self.solve_mut.release()
        return False

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
    # goal = [1.9200621805916276, 0.7833069827304908, -2.946847933613313, -1.049905986025208, -0.2681323848713584, -0.8019316000768786, 1.8069971438800954]
    goal = [1.0656882781191814, 0.4604144797878419, -3.118373350993719, -1.2922323399336983, -0.1051192292831713, -1.3377377734798825, 1.806642277664842]
    comoto_srv = rospy.ServiceProxy("/comoto", Solve)
    rospy.sleep(0.05)
    manager = TrajectoryManager(goal, WEIGHTS, comoto_srv)
    rospy.Subscriber("/right_arm/base_feedback/joint_state", JointState, manager.joint_state_cb)
    rospy.Subscriber("/prediction_traj", JointTrajectory, manager.joint_pred_cb)
    # replan as fast as possible (replan exits if another replan is happening)
    rospy.Timer(rospy.Duration(0.01), manager.mpc_replan)
    rospy.spin()