#!/usr/bin/env python3

"""
Adaptive joint-space trajectory controller for the jaco 2

On new JointTrajectory message, replaces the tail of the existing 
trajectory with the new trajectory.

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
from kinova_msgs.msg import JointVelocity

CTRL_DT = 1/40 # 40Hz max control rate
JOINT_TOL = 0.05 # waypoint tolerance
SPLINE_DT = 1/10 # time elapsed between splined points
VEL_LIMIT = 50*np.array([1.7, 1.7, 1.7, 1.7, 1.7, 3.0, 3.0])
DVEL_LIMIT = 50*0.025

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
        
        self.joint_pos = None
        self.joint_vel = None
        self.joint_errs = []
        self.last_upd_time = None
        
        self.mutex = Lock()
        self.p_gain = 85.5
        self.i_gain = 5.27
        self.i_val = np.zeros(7)
        self.d_gain = 0 # -0.02

        self.ctrl_mut = Lock()
        self.next_ctrl = None
        self.timer = None

    def publish(self, _event):
        """
        Publish self.next_ctrl to the joint velocity controller
        Needs to be called at 100 Hz while control is active 
        """
        # print("Publish called")
        if self.next_ctrl is None:
            # print("No next ctrl")
            return
        
        self.ctrl_mut.acquire()
        
        if self.last_ctrl is None:
            self.last_ctrl = self.next_ctrl
            self.send_vel(self.next_ctrl)
        else:
            diff = self.next_ctrl - self.last_ctrl
            diff = np.clip(diff, -DVEL_LIMIT, DVEL_LIMIT)
            new_ctrl = self.last_ctrl + diff
            self.last_ctrl = new_ctrl
            self.send_vel(new_ctrl)
            
        self.ctrl_mut.release()
            
    def send_vel(self, vel):
        msg = JointVelocity()
        msg.joint1 = vel[0]
        msg.joint2 = vel[1]
        msg.joint3 = vel[2]
        msg.joint4 = vel[3]
        msg.joint5 = vel[4]
        msg.joint6 = vel[5]
        msg.joint7 = vel[6]
        self.vel_pub.publish(msg)

    def joint_state_cb(self, x: JointState):
        self.mutex.acquire()
        now = time()
        
        # update state        
        pos = np.array(x.position)[:7]
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
        self.ctrl_mut.acquire()
        
        if self.last_ctrl is not None:
            keep_latch = self.control_latched & \
                        (np.abs(self.last_ctrl) > np.abs(ctrl)) & \
                        (np.sign(self.last_ctrl) == np.sign(ctrl))
            self.control_latched = keep_latch
            ctrl = np.where(self.control_latched, self.last_ctrl, ctrl)
        
        # periodic debug statements
        if self.steps_since_traj < 1000 and self.steps_since_traj % 10 == 1:
            print("Mask: ", self.control_latched)
            print(f"{self.p_gain}*{err} + {self.i_gain}*{self.i_val} = {ctrl}")
            print()
        
        self.next_ctrl = ctrl
        self.ctrl_mut.release()
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
        self.ctrl_mut.acquire()
        if self.last_ctrl is not None:
            self.control_latched = np.ones(7, dtype=bool)
        self.ctrl_mut.release()
        
        if self.timer is None:
            self.timer = rospy.Timer(rospy.Duration(0.01), self.publish)
        
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

        if self.timer is not None:
            self.timer.shutdown()
            self.timer = None
        
        self.send_vel(np.zeros(7))
        
if __name__ == '__main__':
    np.set_printoptions(precision=4)
    rospy.init_node("jaco_adapt_ctrl", anonymous=True)
    arm_pub = rospy.Publisher(f"/j2s7s300_driver/in/joint_velocity", JointVelocity, queue_size=1)
    controller = JacoAdaptiveCtrl(arm_pub)
    # add callbacks
    rospy.Subscriber(f"/j2s7s300_driver/out/joint_state", JointState, controller.joint_state_cb)
    rospy.Subscriber(f"/jaco_adaptive_ctrl/goal", JointTrajectory, controller.new_traj_cb)
    rospy.Subscriber(f"/jaco_adaptive_ctrl/stop", Empty, controller.stop_cb)    
    
    rospy.spin()
