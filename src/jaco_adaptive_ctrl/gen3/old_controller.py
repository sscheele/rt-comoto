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
VEL_LIMIT = np.array([1.7, 1.7, 1.7, 1.7, 1.7, 3.0, 3.0])
DVEL_LIMIT = 0.025

class JacoAdaptiveCtrl:
    def __init__(self, vel_pub: rospy.Publisher):
        self.requested_traj = np.empty((0,7))
        self.requested_traj_times = np.array([])
        self.spline = None
        self.steps_since_traj = 0
        
        self.control_latched = np.zeros(7, dtype=bool)
        self.last_ctrl = None
        self.d_raw = None
        self.last_err = None
        
        self.vel_pub = vel_pub
        
        self.next_ctrl_time = time()
        self.joint_pos = None
        self.joint_vel = None
        self.joint_errs = []
        self.last_upd_time = None
        
        self.mutex = Lock()
        self.p_gain = 2.5
        self.i_gain = 0.27
        self.i_val = np.zeros(7)
        self.d_gain = -0.02
        
    def rollout_ctrl(self, ctrl):
        """
        Rolls out a control command, possibly limiting change from last control 
        """
        if self.last_ctrl is None:
            self.last_ctrl = ctrl
            self.send_vel(ctrl)
        else:
            diff = ctrl - self.last_ctrl
            diff = np.clip(diff, -DVEL_LIMIT, DVEL_LIMIT)
            new_ctrl = self.last_ctrl + diff
            self.last_ctrl = new_ctrl
            self.send_vel(new_ctrl)
            
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
            
        self.steps_since_traj += 1
        # calculate control values
        # first attempt: try completely relying on velocity controller
        goal = self.spline(now + 0.05)
        err = goal - self.joint_pos
        self.joint_errs.append(np.linalg.norm(err))
        if t_elapsed is not None:
            self.i_val = self.i_val + t_elapsed*err
        
        
        raw_ctrl = self.p_gain*err + self.i_gain*self.i_val
        if self.last_err is not None:
            raw_ctrl += self.d_gain*(err - self.last_err)/t_elapsed
        self.last_err = err
        ctrl = np.clip(raw_ctrl, -VEL_LIMIT, VEL_LIMIT)
            
        # stopgap to prevent jerkiness: repeat last control value
        # after trajectory update until error gets high enough for
        # the controller to work (hopefully)
        if self.last_ctrl is not None:
            keep_latch = self.control_latched & \
                        (np.abs(self.last_ctrl) > np.abs(ctrl)) & \
                        (np.sign(self.last_ctrl) == np.sign(ctrl))
            self.control_latched = keep_latch
            ctrl = np.where(self.control_latched, self.last_ctrl, ctrl)
        
        if self.steps_since_traj < 1000 and self.steps_since_traj % 10 == 1:
            print("Mask: ", self.control_latched)
            print(f"{self.p_gain}*{err} + {self.i_gain}*{self.i_val} = {ctrl}")
            print()
        
        self.rollout_ctrl(ctrl)
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
        
        self.steps_since_traj = 0
        if self.last_ctrl is not None:
            self.control_latched = np.ones(7, dtype=bool)
        print("*** NEW TRAJ ***")
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
        self.last_ctrl = None
        self.last_err = None
        self.requested_traj = np.empty((0,7))
        self.requested_traj_times = np.array([])
        
        self.spline = None        
        self.send_vel(np.zeros(7))
        
if __name__ == '__main__':
    np.set_printoptions(precision=4)
    valid_arms = ['right', 'left']
    arm = 'right'
    # NOTE: commenting because positional command-line arguments get messed up
    # by roslaunch :(
    # if len(sys.argv) > 1:
    #     if sys.argv[1] in valid_arms:
    #         arm = sys.argv[1]
    #     else:
    #         print("Valid arguments are 'left' and 'right'")
    #         exit(1)
    # print(f"Using {arm} arm!")
    rospy.init_node("jaco_adapt_ctrl", anonymous=True)
    arm_pub = rospy.Publisher(f"/{arm}_arm/in/joint_velocity", Base_JointSpeeds, queue_size=1)
    controller = JacoAdaptiveCtrl(arm_pub)
    # add callbacks
    rospy.Subscriber(f"/{arm}_arm/base_feedback/joint_state", JointState, controller.joint_state_cb, queue_size=1)
    rospy.Subscriber("/jaco_adaptive_ctrl/goal", JointTrajectory, controller.new_traj_cb, queue_size=1)
    rospy.Subscriber("/jaco_adaptive_ctrl/stop", Empty, controller.stop_cb, queue_size=1)
    
    rospy.spin()
