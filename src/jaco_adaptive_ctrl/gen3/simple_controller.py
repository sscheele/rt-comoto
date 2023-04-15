#!/usr/bin/env python3

"""
Adaptive joint-space trajectory controller for the jaco Gen3

On new JointTrajectory message, replaces the tail of the existing 
trajectory with the new trajectory. time_from_start in joint
trajectory indexes from time each message is recieved.

On new Empty message sent to cancel topic, will clear trajectories
and stop robot    
"""

import rospy
import numpy as np
from time import time
from threading import Lock
from scipy.interpolate import CubicSpline, PchipInterpolator

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
from kortex_driver.msg import Base_JointSpeeds, JointSpeed

# import matplotlib.pyplot as plt

CTRL_DT = 1/40 # 40Hz max control rate
VEL_LIMIT = 0.3*np.ones(7)
STOP_TIME = 1.7
DVEL_LIMIT = np.max(VEL_LIMIT)/(STOP_TIME*(1/CTRL_DT))

P_GAIN = 9.0

def js_sub(a, b):
    """
    Returns minimum-maginitude difference between a and b 
    in a 2pi symmetric space
    """
    # since values are in (-pi, pi), we can simply add/subtract 2*pi
    # iff we're more than pi away from the ref
    diff = a - b
    while np.any(diff < -np.pi):
        diff[diff < -np.pi] += 2*np.pi
    while np.any(diff > np.pi):
        diff[diff > np.pi] -= 2*np.pi
    return diff

class JacoAdaptiveCtrl:
    def __init__(self, vel_pub: rospy.Publisher):
        self.traj_mut = Lock()
        self.requested_traj = np.empty((0,7))
        self.requested_traj_times = np.array([])
        self.spline = None
        
        self.vel_pub = vel_pub
        
        self.error = None
        self.next_ctrl = None
        self.last_ctrl = None
        self.next_ctrl_mut = Lock()
        
        self.joint_state_mut = Lock()
        self.joint_pos = None
        self.joint_vel = None
        
        self.p_gain = P_GAIN
        
    def rollout_ctrl_cb(self, evt):
        """
        Timer callback to rollout a control
        """
        self.next_ctrl_mut.acquire()
        if self.next_ctrl is None:
            self.next_ctrl_mut.release()
            return
        
        new_ctrl = self.next_ctrl
        if self.last_ctrl is not None:
            diff = self.next_ctrl - self.last_ctrl
            diff = np.clip(diff, -DVEL_LIMIT, DVEL_LIMIT)
            new_ctrl = self.last_ctrl + diff

        new_ctrl = np.clip(new_ctrl, -VEL_LIMIT, VEL_LIMIT)
        
        self.last_ctrl = new_ctrl      
        self.send_vel(new_ctrl)
        self.next_ctrl_mut.release()
            
    def send_vel(self, vel):
        # JointSpeed has u32 joint_identifier, f32 value, u32 duration
        msg = Base_JointSpeeds()
        for i in range(len(vel)):
            tmp = JointSpeed()
            tmp.joint_identifier = i
            tmp.value = vel[i]
            tmp.duration = 1 # TODO: change?
            msg.joint_speeds.append(tmp)
        msg.duration = 1
        self.vel_pub.publish(msg)

    def joint_state_cb(self, x: JointState):
        # update state        
        self.joint_state_mut.acquire()
        pos = np.array(x.position)[:7]
        self.joint_pos = np.copy(pos)
        self.joint_vel = np.array(x.velocity)[:7]
        self.joint_state_mut.release()
        
        now = time()
        
        self.traj_mut.acquire()
        # if trajectories are cleared, return with no command
        if len(self.requested_traj_times) == 0:
            self.traj_mut.release()
            return
            
        # if out of time, cancel
        if now >= self.requested_traj_times[-1]:
            self.cancel()
            self.traj_mut.release()
            return
        splined_goal = self.spline(now)
        # calculate control values
        # first attempt: try completely relying on velocity controller
        # goal = splined_goal
        self.traj_mut.release()
        
        self.error = js_sub(splined_goal, pos)
        next_ctrl = self.p_gain*self.error

        self.next_ctrl_mut.acquire()
        self.next_ctrl = next_ctrl
        self.next_ctrl_mut.release()
            
    def new_traj_cb(self, traj: JointTrajectory):
        if len(traj.points) == 0:
            return
        self.joint_state_mut.acquire()
        if self.joint_vel is None:
            print("Ignoring trajectory because we don't have a joint vel")
            self.joint_state_mut.release()
            return
        curr_pos = np.copy(self.joint_pos)
        curr_vel = np.copy(self.joint_vel)
        self.joint_state_mut.release()

        # destructure into lists of times and states
        curr_time = time()
        traj_times = np.array([
            curr_time + pt.time_from_start.to_sec() 
            for pt in traj.points])
        traj_posns = np.array([x.positions for x in traj.points])
        
        self.traj_mut.acquire()
        # prune past times from requested traj
        if len(self.requested_traj_times) > 0:
            now_idx = np.searchsorted(self.requested_traj_times, curr_time)
            self.requested_traj_times = self.requested_traj_times[now_idx:]
            self.requested_traj = self.requested_traj[now_idx:, :]
        
        # find insertion point
        ins_idx = np.searchsorted(self.requested_traj_times, traj_times[0])
        if ins_idx == 0:
            self.requested_traj_times = traj_times
            self.requested_traj = traj_posns
        else:
            self.requested_traj_times = np.concatenate((
                self.requested_traj_times[:ins_idx],
                traj_times
            ))
            self.requested_traj = np.concatenate((
                self.requested_traj[:ins_idx, :],
                traj_posns
            ), axis=0)
            
        # put traj in a canonical form with 0 at start
        if self.requested_traj_times[0] != curr_time:
            self.requested_traj_times = np.concatenate((
                np.array([curr_time]),
                self.requested_traj_times
            ), axis=0)
            self.requested_traj = np.concatenate((
                np.array([curr_pos]), 
                self.requested_traj
            ), axis=0)
        
            if self.error is not None:
                self.requested_traj[0] += self.error
                
        if len(self.requested_traj_times) < 2:
            self.cancel()
            self.traj_mut.release()
            return
            
        for i in range(1, self.requested_traj.shape[0]):
            self.requested_traj[i] = self.requested_traj[i-1] + \
                js_sub(self.requested_traj[i], self.requested_traj[i-1])
            
        # self.spline = CubicSpline(self.requested_traj_times, self.requested_traj, 
        #     bc_type=((1, curr_vel),(1, np.zeros(7))))
        
        self.spline = PchipInterpolator(self.requested_traj_times, 
            self.requested_traj, extrapolate=False)
                
        # plt.plot((3/500)*np.arange(500), self.spline(curr_time + (3/500)*np.arange(500))[:,0])
        
        # interp = np.interp(
        #     curr_time + (3/500)*np.arange(500),
        #     self.requested_traj_times,
        #     self.requested_traj[:,0]
        # )
        # plt.plot((3/500)*np.arange(500), interp)
        # plt.show()
        
        self.traj_mut.release()
                
    def stop_cb(self, msg: Empty):
        """
        Stop callback - acquires mutex to avoid messing with other callbacks    
        """
        self.traj_mut.acquire()
        self.next_ctrl_mut.acquire()
        # print("Average joint error since last cancel: ", np.mean(self.joint_errs))
        # self.joint_errs = []
        self.cancel()
        self.next_ctrl_mut.release()
        self.traj_mut.release()
    
    def cancel(self):
        """
        Perform stop - does not acquire mutex 
        """
        self.last_ctrl = None
        self.next_ctrl = None
        self.requested_traj = np.empty((0,7))
        self.requested_traj_times = np.array([])
        self.spline = None
        self.error = None
        
        self.send_vel(np.zeros(7))
        
if __name__ == '__main__':
    np.set_printoptions(precision=4)
    valid_arms = ['right', 'left']
    arm = 'right'
    rospy.init_node("jaco_adapt_ctrl", anonymous=True)
    arm_pub = rospy.Publisher(f"/{arm}_arm/in/joint_velocity", Base_JointSpeeds, queue_size=1)
    controller = JacoAdaptiveCtrl(arm_pub)

    # add callbacks
    rospy.Subscriber(f"/{arm}_arm/base_feedback/joint_state", JointState, controller.joint_state_cb, queue_size=1)
    rospy.Subscriber("/jaco_adaptive_ctrl/goal", JointTrajectory, controller.new_traj_cb, queue_size=1)
    rospy.Subscriber("/jaco_adaptive_ctrl/stop", Empty, controller.stop_cb, queue_size=1)
    rospy.Timer(rospy.Duration(CTRL_DT), controller.rollout_ctrl_cb)

    rospy.spin()
