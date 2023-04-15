#!/usr/bin/env python3
import rospy
import numpy as np
import kinpy as kp
import os.path
import sys
import pickle

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from baselines import positions
from baselines.nominal import cart2jnt

from time import time

PKG_DIR = os.path.dirname(__file__)
TRAJ_DIR = os.path.join(PKG_DIR, 'leg_trajs')
LEG_DT = 0.01

jaco_ee_link = "EndEffector_Link"
urdf_path = os.path.join(PKG_DIR, 'gen3.urdf')
gen3_chain = kp.build_serial_chain_from_urdf(open(urdf_path).read(), jaco_ee_link)

def jaco_ee_fk(posn):
    ee_trans = gen3_chain.forward_kinematics(posn)
    return np.asarray(ee_trans.pos)
    
def get_y_min(cart_traj, ax=0.56, ay=0.67):
    x = cart_traj[:,0]
    y_max = ay * np.sqrt(
        1 - np.power(x/ax, 2)
    )
    y_max = np.minimum(y_max, 0.58)
    return -y_max
    
def get_z_max(cart_traj, ax=1, ay=0.64, az=0.23):
    x = cart_traj[:,0]
    y = cart_traj[:,1]
    # print('x: ', x)
    # print('y: ', y)
    # print(1 - np.power(x/ax, 2) - np.power(y/ay, 2))
    z_max = az * (np.sqrt(
        1 - np.power(x/ax, 2) - np.power(y/ay, 2)
    ))
    return 0.33 + z_max

class Legible:
    def __init__(self, joint_goal):
        self.reached_goal = False
        self.sent_linear_cmd = False
        self.is_torn_down = False

        self.traj_pub = rospy.Publisher('/jaco_adaptive_ctrl/goal', 
            JointTrajectory, queue_size=1)

        self.goal = np.asarray(joint_goal)
        self.cart_goal = jaco_ee_fk(self.goal)
        
        curr_posn = rospy.wait_for_message(
            '/right_arm/base_feedback/joint_state', JointState)
        curr_posn = np.array(curr_posn.position[:7])
        curr_cart_posn = jaco_ee_fk(curr_posn)
        
        all_posns = np.concatenate((
            positions.home.reshape((1, 7)),
            positions.posns
        ), axis=0)

        self.goal_idx = np.where(
                np.all(all_posns == joint_goal, axis=1)
            )[0][0]
        
        all_cart_posns = np.array([jaco_ee_fk(x) for x in all_posns])
        curr_dists = np.linalg.norm(all_cart_posns - curr_cart_posn, axis=1).ravel()
        curr_idx = np.argmin(curr_dists)
        if curr_idx == self.goal_idx:
            self.reached_goal = True
            return
        if curr_dists[curr_idx] > 0.1:
            raise ValueError(f"Not at any of the pegs! Closest: {curr_idx} " +
                f"at {curr_dists[curr_idx]}")
        else:
            print(f"Navigating from {curr_idx} to {self.goal_idx}")
            
        
        traj_fname = os.path.join(TRAJ_DIR, f"{curr_idx}-{self.goal_idx}.pkl")
        print('Opening: ', traj_fname)
        with open(traj_fname, 'rb') as f:
            cart_traj = pickle.load(f)
            cart_traj += curr_cart_posn
            cart_traj[:,0] = np.clip(cart_traj[:,0], -0.47, 0.45)
            y_min = get_y_min(cart_traj)
            cart_traj[:,1] = np.clip(cart_traj[:,1], y_min, -0.2)
            # non-singular space is approximately cylindrical in our ranges
            z_max = get_z_max(cart_traj)
            cart_traj[:,2] = np.clip(cart_traj[:,2], 0.33, z_max)
            traj, traj_times = cart2jnt(cart_traj, curr_posn, self.goal, curr_idx==0)
        # print("Solved traj: ")
        # for p in traj[::8,:]:
        #     print(jaco_ee_fk(p))
        traj_times = 1.8*traj_times
        if len(traj_times) > 1:
            traj_times[-1] = traj_times[-2] + (2)*(traj_times[-1] - traj_times[-2])
        else:
            traj_times[-1] = 2*traj_times[-1]
        
        self.traj_dur = traj_times[-1]
        
        msg = JointTrajectory()
        for i in range(traj.shape[0]):
            pt = JointTrajectoryPoint()
            pt.time_from_start = rospy.Duration(traj_times[i])
            pt.positions = list(traj[i,:])
            msg.points.append(pt)
        
        rospy.sleep(0.1)
        self.traj_pub.publish(msg)
        rospy.sleep(0.1)
        self.t_start = time()

        self.pub_subs = [
            self.traj_pub,
            rospy.Subscriber('/right_arm/base_feedback/joint_state', JointState,
                self.js_cb, queue_size=1)
        ]
        
        
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
        elif time() - self.t_start > self.traj_dur:
            if lin_dist > 0.2:
                print("Couldn't get close to goal!")
                self.teardown()
            elif not self.sent_linear_cmd:
                self.send_linear_command()
                print("Driving the rest of the way to goal")
        
        if lin_dist < 0.15 and not self.sent_linear_cmd:
            self.send_linear_command()
            print("Driving to goal...")
                
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
    
        
if __name__ == "__main__":
    rospy.init_node("legible", anonymous=True)
    goal = positions.posns[int(sys.argv[1])]
    man = Legible(goal)
    rospy.spin()    