#!/usr/bin/env python

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
from scipy.interpolate import CubicSpline
from threading import Lock
import sys

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
from kortex_driver.msg import Base_JointSpeeds, JointSpeed

CTRL_DT = 1/40 # 40Hz max control rate
JOINT_TOL = 0.05 # waypoint tolerance
SPLINE_DT = 1/10 # time elapsed between splined points
ACC_LIMIT = 3.0
VEL_LIMIT = 0.3

class JacoAdaptiveCtrl:
    def __init__(self, vel_pub: rospy.Publisher):
        self.requested_traj = np.empty((0,7))
        self.requested_traj_times = np.array([])
        self.spline = None

        self.vel_pub = vel_pub
        
        self.next_ctrl_time = time()
        self.joint_pos = None
        self.joint_vel = None
        self.joint_errs = []
        self.last_upd_time = None
        
        self.mutex = Lock()
        self.p_gain = 1.2
        self.i_gain = 0
        self.i_val = np.zeros(7)
        
        self.vel_integ = np.zeros(7)
        
    def rollout_ctrl(self, acc, dt):
        """
        Internally integrates accelerations to send to the velocity
        controller - this is dumb but I don't really want to deal
        with it 
        """
        self.vel_integ += dt*acc
        self.vel_integ = np.clip(self.vel_integ, -VEL_LIMIT, VEL_LIMIT)
        self.send_vel(self.vel_integ)
    
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
        self.mutex.acquire()
        now = time()
        
        # ensure controller rate
        if now < self.next_ctrl_time:
            self.mutex.release()
            return
        self.next_ctrl_time = now + CTRL_DT
        
        # update state        
        pos = np.array(x.position)[:-1]
        t_elapsed = None
        if self.last_upd_time is not None:
            t_elapsed = now - self.last_upd_time
            self.joint_vel = (pos - self.joint_pos)/(now - self.last_upd_time)
        self.last_upd_time = now
        self.joint_pos = pos

        # no requested traj, return
        if len(self.requested_traj_times) == 0:
            self.mutex.release()
            return
        
        # if trajectories are cleared, return with no command
        if self.spline is None:
            self.mutex.release()
            return
        # if exceeded traj bounds and trajectories not yet cleared
        # cancel and return (sends zero vel)
        if now > self.requested_traj_times[-1]:
            self.cancel()
            self.mutex.release()
            return

        # calculate control values
        # first attempt: try completely relying on velocity controller
        goal = self.spline(now)
        err = goal - self.joint_pos
        self.joint_errs.append(np.linalg.norm(err))
        
        if t_elapsed is not None:
            self.i_val = self.i_val + t_elapsed*err
        
        ctrl = self.p_gain*err + self.i_gain*self.i_val
        ctrl = np.clip(ctrl, -ACC_LIMIT, ACC_LIMIT)
        self.rollout_ctrl(ctrl, t_elapsed) 
        self.mutex.release()        
            
    def new_traj_cb(self, traj: JointTrajectory):
        self.mutex.acquire()
        if self.joint_vel is None:
            print("Ignoring trajectory because we don't have a joint vel")
            self.mutex.release()
            return
        # destructure into lists of times and states
        curr_time = time()
        traj_times = np.array([
            curr_time + pt.time_from_start.to_sec() 
            for pt in traj.points])
        traj_posns = np.array([x.positions for x in traj.points])
        
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
            
        # generate spline data by possibly prepending current time/state
        if self.requested_traj_times[0] == curr_time:
            self.spline = CubicSpline(self.requested_traj_times, self.requested_traj, bc_type=(
                (1, self.joint_vel),(1, np.zeros(7))))
        else:
            self.spline = CubicSpline(
                np.concatenate((
                    np.array([curr_time]),
                    self.requested_traj_times
                )),
                np.concatenate((
                    np.array([self.joint_pos]),
                    self.requested_traj
                ), axis=0),
                bc_type=((1, self.joint_vel),(1, np.zeros(7)))
            )
        self.mutex.release()
        # no need to update here - will be done in joint_state_cb    
                
    def stop_cb(self, msg: Empty):
        """
        Stop callback - acquires mutex to avoid messing with other callbacks    
        """
        self.mutex.acquire()
        print("Average joint error since last cancel: ", np.mean(self.joint_errs))
        self.joint_errs = []
        self.cancel()
        self.mutex.release()
    
    def cancel(self):
        """
        Perform stop - does not acquire mutex 
        """
        self.i_val = np.zeros(7)
        self.requested_traj = np.empty((0,7))
        self.requested_traj_times = np.array([])
        
        self.spline = None        
        self.send_vel(np.zeros(7))
        
if __name__ == '__main__':
    valid_arms = ['right', 'left']
    arm = 'right'
    if len(sys.argv) > 1:
        if sys.argv[1] in valid_arms:
            arm = sys.argv[1]
        else:
            print("Valid arguments are 'left' and 'right'")
            exit(1)
    print(f"Using {arm} arm!")
    rospy.init_node("jaco_adapt_ctrl", anonymous=True)
    arm_pub = rospy.Publisher(f"/{arm}_arm/in/joint_velocity", Base_JointSpeeds)
    controller = JacoAdaptiveCtrl(arm_pub)
    # add callbacks
    rospy.Subscriber(f"/{arm}_arm/base_feedback/joint_state", JointState, controller.joint_state_cb)
    rospy.Subscriber(f"/{arm}_arm/jaco_adaptive_ctrl/goal", JointTrajectory, controller.new_traj_cb)
    rospy.Subscriber(f"/{arm}_arm/jaco_adaptive_ctrl/stop", Empty, controller.stop_cb)    
    
    rospy.spin()
