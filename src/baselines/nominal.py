#!/usr/bin/env python3
import rospy
import numpy as np
import kinpy as kp
import os.path
import scipy
from time import time
import sys

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import transformations as tf

from baselines import positions

PKG_DIR = os.path.dirname(__file__)
NOM_DT = 0.01

jaco_ee_link = "EndEffector_Link"
urdf_path = os.path.join(PKG_DIR, 'gen3.urdf')
gen3_chain = kp.build_serial_chain_from_urdf(open(urdf_path).read(), jaco_ee_link)

def jaco_ee_fk(posn):
    ee_trans = gen3_chain.forward_kinematics(posn)
    return np.asarray(ee_trans.pos)

def jaco_ee_rot(posn):
    ee_trans = gen3_chain.forward_kinematics(posn)
    rot_quat = np.asarray(ee_trans.rot)
    rot_euler = tf.euler_from_quaternion(rot_quat, 'rxyz')
    return np.asarray(rot_euler)

class Nominal:
    def __init__(self, joint_goal):
        self.goal = np.asarray(joint_goal)
        self.cart_goal = jaco_ee_fk(self.goal)
        curr_posn = rospy.wait_for_message(
            '/right_arm/base_feedback/joint_state', JointState)
        curr_posn = np.array(curr_posn.position[:7])
        self.posn = curr_posn

        self.traj_pub = rospy.Publisher('/jaco_adaptive_ctrl/goal', 
            JointTrajectory, queue_size=None, tcp_nodelay=True)
            
        cart_traj = np.array([jaco_ee_fk(curr_posn), jaco_ee_fk(self.goal)])
        self.traj, times = cart2jnt(cart_traj, curr_posn, self.goal, rot_ctrl=True)
        times = 2.7*times
        
        msg = JointTrajectory()
        for i in range(self.traj.shape[0]):
            pt = JointTrajectoryPoint()
            pt.time_from_start = rospy.Duration(times[i])
            pt.positions = list(self.traj[i,:])
            msg.points.append(pt)
        
        rospy.sleep(0.1)
        self.traj_pub.publish(msg)
        self.t_start = time()
        self.traj_dur = msg.points[-1].time_from_start.to_sec()
        print("Traj lasts %f sec"%self.traj_dur)
        rospy.sleep(0.05)

        self.is_torn_down = False

        self.pub_subs = [
            self.traj_pub,
            rospy.Subscriber('/right_arm/base_feedback/joint_state', JointState,
                self.js_cb, queue_size=1)
        ]
        
        self.sent_linear_cmd = False
        self.reached_goal = False
        
    def teardown(self):
        if self.is_torn_down:
            return
        self.is_torn_down = True
        for x in self.pub_subs:
            x.unregister()
        
    def js_cb(self, state: JointState):
        pos = np.array(state.position[:7])
        self.posn = pos
        lin_dist = np.linalg.norm(jaco_ee_fk(pos) - self.cart_goal)
        # print(js_dist)
        if lin_dist < 0.05:
            print("Reached goal")
            self.reached_goal = True
            self.teardown()
        elif lin_dist < 0.15 and not self.sent_linear_cmd:
            self.send_linear_command()
            print("Driving to goal...")
        elif time() - self.t_start > self.traj_dur:
            if lin_dist > 0.2:
                print("Couldn't get close to goal!")
                self.teardown()
            elif not self.sent_linear_cmd:
                self.send_linear_command()
                print("Driving the rest of the way to goal")
        
                
    def send_linear_command(self):
        self.sent_linear_cmd = True
        pt0 = JointTrajectoryPoint()
        pt0.positions = list(self.posn)
        pt0.time_from_start = rospy.Duration(0)
        
        pt = JointTrajectoryPoint()
        pt.positions = list(self.goal)
        max_jt_dst = np.max(np.abs(js_sub(self.goal, self.posn)))
        pt.time_from_start = rospy.Duration(max_jt_dst/0.15)
        msg = JointTrajectory()
        msg.points = [pt0, pt]
        self.traj_pub.publish(msg)
        rospy.sleep(0.1)

class JSNominal:
    def __init__(self, joint_goal):
        goal = np.asarray(joint_goal)
        self.cart_goal = jaco_ee_fk(goal)
        traj_pub = rospy.Publisher('/jaco_adaptive_ctrl/goal', 
            JointTrajectory, queue_size=None, tcp_nodelay=True)
        rospy.sleep(0.5)
        
        posn = rospy.wait_for_message(
            '/right_arm/base_feedback/joint_state', JointState)
        posn = np.array(posn.position[:7])
        pt0 = JointTrajectoryPoint()
        pt0.positions = list(posn)
        pt0.time_from_start = rospy.Duration(0)
        
        pt = JointTrajectoryPoint()
        pt.positions = list(goal)
        lin_dist = np.linalg.norm(jaco_ee_fk(posn) - self.cart_goal)
        pt.time_from_start = rospy.Duration(lin_dist/0.05)
        msg = JointTrajectory()
        msg.points = [pt0, pt]
        traj_pub.publish(msg)
        rospy.sleep(0.5)
        self.pub_subs = [
            rospy.Subscriber('/right_arm/base_feedback/joint_state', JointState,
                self.js_cb, queue_size=1)
        ]
        
        self.reached_goal = False
        
    def teardown(self):
        for ps in self.pub_subs:
            ps.unregister()
        
    def js_cb(self, state: JointState):
        pos = np.array(state.position[:7])
        lin_dist = np.linalg.norm(jaco_ee_fk(pos) - self.cart_goal)
        # print(js_dist)
        if lin_dist < 0.05:
            print("Reached goal")
            self.reached_goal = True
            self.teardown()
    
def with_max_mag(vec, mag):
    vec_norm = np.linalg.norm(vec)
    if vec_norm < mag:
        return vec
    return (mag/vec_norm)*vec
    
def js_sub(vec1, vec2):
    diff = vec1 - vec2
    diff[diff > np.pi] -= 2*np.pi
    diff[diff < -np.pi] += 2*np.pi
    return diff
    
def cart2jnt(cart_traj, joint_start, joint_goal, rot_ctrl=False, cart_tol=0.05):
    """
    cart_traj: (ts, 3) a cartesian-space ee trajectory to follow
    joint_start: (7) starting joint-space position (must have 
        ee_fk(joint_start) == cart_traj[0,:])
    joint_goal: (7) ending joint-space position (must have
        ee(fk(joint_goal) == cart_traj[-1,:]))
        
    Returns: (ts, 7) joint-space trajectory from joint_start to joint_goal
        that follows cart_traj with the end effector 
        (ts) array of times
    """
    fake_goal = joint_start + js_sub(joint_goal, joint_start)
    traj = cart2jnt_hlpr(cart_traj, joint_start, fake_goal, rot_ctrl, cart_tol)
    times = NOM_DT*np.arange(len(traj))
    if len(traj) == 0:
        return np.array([fake_goal]), np.array([5])
    
    if not np.all(traj[-1] == fake_goal):
        traj.append(fake_goal)
        for i in range(1, len(traj)):
            traj[i] = traj[i-1] + js_sub(traj[i], traj[i-1])
        times = np.concatenate((times, [times[-1] + \
            np.max(np.abs(fake_goal - traj[-2]))/0.3]))
    
    return np.array(traj), times

def cart2jnt_hlpr(cart_traj, joint_start, joint_goal, rot_ctrl=False, cart_tol=0.05):
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
    max_cart_vel = 0.1
    max_jnt_vel = 0.2
    max_angular_vel = 0.25
    
    rot_goal = jaco_ee_rot(joint_goal)
    
    next_traj_idx = 0
    while True:
        traj.append(curr_state)
        # check for completed waypoints and get next state
        curr_cart_posn = jaco_ee_fk(curr_state)
        next_cart_pt = cart_traj[next_traj_idx,:]
        while np.linalg.norm(curr_cart_posn - next_cart_pt) < cart_tol:
            next_traj_idx += 1
            if next_traj_idx == cart_traj.shape[0]:
                return traj
            next_cart_pt = cart_traj[next_traj_idx,:]
                            
        lin_vel = with_max_mag(next_cart_pt - curr_cart_posn, max_cart_vel)
        # find robot jacobian
        J = gen3_chain.jacobian(curr_state)
        # inverse jacobian will let us go from cart vel to joint vel
        J_inv = np.linalg.pinv(J)
        # multiply robot direction by J_inv to get translation component
        oriented_cart_vel = np.zeros(6)
        oriented_cart_vel[:3] = lin_vel
        if rot_ctrl:
            curr_rot = jaco_ee_rot(curr_state)
            angular_vel = with_max_mag(js_sub(rot_goal, curr_rot), max_angular_vel)
            oriented_cart_vel[3:] = angular_vel
        curr_vel = np.matmul(J_inv, oriented_cart_vel).ravel()
        
        joint_dir = with_max_mag(joint_goal - curr_state, max_jnt_vel)
        # find the null space of the Jacobian to see vectors that can get us
        # close jointwise
        # we can ignore the orientation part of the Jacobian, since JS moves
        # towards the final configuration should get us closer to the desired
        # orientation
        J_null = scipy.linalg.null_space(J[:3,:])
        # project JS-linear vector into the null space
        J_null_inv = np.linalg.pinv(J_null)
        null_weights = np.matmul(J_null_inv, joint_dir)
        # get the null vector closest to the JS-linear vector and add to vel
        null_proj = np.matmul(J_null, null_weights)
        curr_vel += null_proj.ravel()
        
        curr_cart_vel = np.linalg.norm(np.matmul(J, curr_vel).ravel()[:3])
        if curr_cart_vel > max_cart_vel:
            curr_vel = (max_cart_vel/curr_cart_vel)*curr_vel
        
        curr_state = curr_state + NOM_DT*curr_vel
        
if __name__ == "__main__":
    rospy.init_node("nominal", anonymous=True)
    # goal = positions.home
    goal = positions.posns[int(sys.argv[1])]
    man = Nominal(goal)
    rospy.spin()