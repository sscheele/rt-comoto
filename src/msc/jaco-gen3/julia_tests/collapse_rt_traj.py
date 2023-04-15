"""
    Listens for partial trajectories from RT-CoMOTO and assembles
    them into a full trajectory suitable for a traditional traj
    controller. When it gets an empty message, prints full trajectory
    and sends it to a controller if applicable 
"""

import numpy as np
import rospy

from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal

class TrajReporter:
    def __init__(self, traj_pub: rospy.Publisher=None):
        self.states = []
        self.times = []
        self.joint_names = []
        self.traj_pub = traj_pub
              
    def handle_traj(self, traj: JointTrajectory):
        traj_states = [x.positions for x in traj.points]
        traj_times = [x.time_from_start for x in traj.points]
        
        print("New message at: ", rospy.Time())
        print("Received states: ", traj_states)
        print("Received times: ", traj_times)
        if len(self.states) == 0:
            self.states = traj_states
            self.times = traj_times
            self.joint_names = traj.joint_names
            return

        ins_idx = np.searchsorted(self.times, traj.points[0].time_from_start)
        
        # remove tail of trajectory where we're inserting the new one
        self.states = self.states[:ins_idx]
        self.times = self.times[:ins_idx]
        
        self.states.extend(traj_states)
        self.times.extend(traj_times)
        print("New program state: ")
        print("States: ", self.states)
        print("Times: ", self.times)
        print("="*20 + "\n")
    
    def handle_exec(self, msg: Empty):
        print("traj_states = ", self.states)
        print("traj_times = ", [x.to_nsec() for x in self.times])
        
        if self.traj_pub is None:
            return
        
        out_goal = FollowJointTrajectoryActionGoal()    
        out_traj = JointTrajectory()
        # TODO use self.joint_names
        out_traj.joint_names = [f"Actuator{i}" for i in range(1,8)]
        for idx, pt in enumerate(self.states):
            traj_point = JointTrajectoryPoint()
            traj_point.time_from_start = self.times[idx]
            traj_point.positions = pt
            out_traj.points.append(traj_point)
        out_goal.goal.trajectory = out_traj
        self.traj_pub.publish(out_goal)
        
if __name__ == "__main__":
    rospy.init_node("collapse_traj")
    traj_pub = rospy.Publisher("/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=1)
    
    rpt = TrajReporter(traj_pub)
    rospy.Subscriber("/right_arm/jaco_adaptive_ctrl/goal", JointTrajectory, rpt.handle_traj, queue_size=1)
    rospy.Subscriber("/right_arm/traj_collapse/go", Empty, rpt.handle_exec)
    rospy.spin()    