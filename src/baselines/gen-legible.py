#!/usr/bin/env python3
import rospy
import numpy as np
import kinpy as kp
import os.path
import scipy
import pickle

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from baselines import positions

import time

PKG_DIR = os.path.dirname(__file__)
TRAJ_DIR = os.path.join(PKG_DIR, 'leg_trajs')
LEG_DT = 0.1

jaco_ee_link = "EndEffector_Link"
urdf_path = os.path.join(PKG_DIR, 'gen3.urdf')
gen3_chain = kp.build_serial_chain_from_urdf(open(urdf_path).read(), jaco_ee_link)

# traj_files should contain filenames for precomputed legible trajectories
TRAJ_FILES = [os.path.join(TRAJ_DIR, 'tmp.pkl')]*8 # TODO

def jaco_ee_fk(posn):
    ee_trans = gen3_chain.forward_kinematics(posn)
    return np.asarray(ee_trans.pos)

def with_max_mag(vec, mag):
    vec_norm = np.dot(vec, vec)
    if np.sqrt(vec_norm) < mag:
        return vec
    return (mag/vec_norm)*vec
    
def js_sub(vec1, vec2):
    diff = vec1 - vec2
    diff[diff > np.pi] -= 2*np.pi
    diff[diff < -np.pi] += 2*np.pi
    return diff
    
def cart2jnt(cart_traj, joint_start, joint_goal):
    """
    cart_traj: (ts, 3) a cartesian-space ee trajectory to follow
    joint_start: (7) starting joint-space position (must have 
        ee_fk(joint_start) == cart_traj[0,:])
    joint_goal: (7) ending joint-space position (must have
        ee(fk(joint_goal) == cart_traj[-1,:]))
        
    Returns: (ts, 7) joint-space trajectory from joint_start to joint_goal
        that follows cart_traj with the end effector 
    """
    fake_goal = joint_start + js_sub(joint_goal, joint_start)
    traj = cart2jnt_hlpr(cart_traj, joint_start, fake_goal)
    if len(traj) == 0:
        return np.array([fake_goal])
    elif not np.all(traj[-1] == fake_goal):
        traj.append(fake_goal)
    return np.array(traj)

def cart2jnt_hlpr(cart_traj, joint_start, joint_goal):
    """
    INTERNAL
    
    cart_traj: (ts, 3) a cartesian-space ee trajectory to follow
    joint_start: (7) starting joint-space position (must have 
        ee_fk(joint_start) == cart_traj[0,:])
    joint_goal: (7) ending joint-space position (must have
        ee(fk(joint_goal) == cart_traj[-1,:]))
        
    Returns: non-np Array of joint-space states from joint_start to 
        somewhere close to joint_goal that follows cart_traj with the 
        end effector
    """
    curr_state = joint_start.copy()
    traj = []
    cart_vel = 0.07
    jnt_vel = 0.25
    
    next_traj_idx = 0
    while True:
        traj.append(curr_state)
        # check for completed waypoints and get next state
        curr_cart_posn = jaco_ee_fk(curr_state)
        next_cart_pt = cart_traj[next_traj_idx,:]
        while np.linalg.norm(curr_cart_posn - next_cart_pt) < 0.05:
            next_traj_idx += 1
            if next_traj_idx == cart_traj.shape[0]:
                return traj
            next_cart_pt = cart_traj[next_traj_idx,:]
                            
        cart_dir = with_max_mag(next_cart_pt - curr_cart_posn, cart_vel)
        # find robot jacobian
        # TODO verify that last items are actually twists
        J = gen3_chain.jacobian(curr_state)[:3,:]
        # inverse jacobian will let us go from cart vel to joint vel
        J_inv = np.linalg.pinv(J)
        # multiply robot direction by J_inv to get translation component
        curr_vel = np.matmul(J_inv, cart_dir).ravel()
        
        joint_dir = with_max_mag(joint_goal - curr_state, jnt_vel)
        # find the null space of the Jacobian to see vectors that can get us
        # close jointwise
        J_null = scipy.linalg.null_space(J)
        # project JS-linear vector into the null space
        J_null_inv = np.linalg.pinv(J_null)
        null_weights = np.matmul(J_null_inv, joint_dir)
        # get the null vector closest to the JS-linear vector and add to vel
        null_proj = np.matmul(J_null, null_weights)
        curr_vel += null_proj.ravel()
        
        curr_state = curr_state + LEG_DT*curr_vel
        
if __name__ == "__main__":
    START_IDX = 1
    END_IDX = 3
    orig = np.array([0.12810357, 0.12573313, 0.12335804, 0.1209791 , 0.1185971 ,
       0.11621278, 0.11382684, 0.11143994, 0.1090527 , 0.1066657 ,
       0.10427948, 0.10189455, 0.09951135, 0.09713032, 0.09475184,
       0.09237625, 0.09000386, 0.08763495, 0.08526975, 0.08290847,
       0.08055127, 0.07819829, 0.07584965, 0.07350542, 0.07116565,
       0.00478646, 0.00476815, 0.00474879, 0.00472839, 0.00470696,
       0.00468451, 0.00466107, 0.00463664, 0.00461125, 0.00458491,
       0.00455765, 0.00452947, 0.00450041, 0.00447049, 0.00443971,
       0.00440812, 0.00437573, 0.00434256, 0.00430863, 0.00427398,
       0.00423862, 0.00420257, 0.00416586, 0.00412851, 0.00409054,
       0.00057154, 0.00058985, 0.00060921, 0.00062961, 0.00065104,
       0.00067349, 0.00069693, 0.00072136, 0.00074675, 0.00077309,
       0.00080035, 0.00082853, 0.00085759, 0.00088751, 0.00091829,
       0.00094988, 0.00098227, 0.00101544, 0.00104937, 0.00108402,
       0.00111938, 0.00115543, 0.00119214, 0.00122949, 0.00126746])
    orig = orig.reshape((3, -1))
    orig = np.concatenate((np.zeros((3, 1)), orig), axis=1)
    cart_traj = 0.2*np.cumsum(orig, axis=1).T
    print("Orig cart: ", cart_traj)
    jnt_start = positions.posns[START_IDX,:]
    jnt_goal = positions.posns[END_IDX,:]
    start_cart = jaco_ee_fk(jnt_start)
    goal_cart = jaco_ee_fk(jnt_goal)
    cart_traj = cart_traj + start_cart
    traj = cart2jnt(cart_traj, jnt_start, jnt_goal)
    print("Desired cart traj: ")
    print(repr(cart_traj))
    print("="*20)
    print("Actual cart traj: ")
    print(repr(np.array([jaco_ee_fk(x) for x in traj])))
    
    msg = JointTrajectory()
    for i, x in enumerate(traj):
        pt = JointTrajectoryPoint()
        pt.positions = x
        pt.time_from_start = rospy.Duration(1.5*i*LEG_DT)
        msg.points.append(pt)
    with open(os.path.join(TRAJ_DIR, f'{START_IDX}-{END_IDX}.pkl'), 'wb') as f:
        pickle.dump(msg, f)