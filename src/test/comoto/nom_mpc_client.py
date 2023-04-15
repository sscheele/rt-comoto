#!/usr/bin/env python3
import numpy as np
import rospy
import kinpy as kp

from math import pi

from threading import Lock
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from julia_comoto.srv import Solve, SolveResponse, SolveRequest

PLAN_TIME = 0.3
PLAN_DT = 0.5

def get_goal_nearest(goal, pos):
    out_goal = np.copy(goal)
    for i in range(len(goal)):
        out = goal[i]
        while abs(out + 2*pi - pos[i]) < abs(out - pos[i]):
            out = out + 2*pi
        while abs(out - 2*pi - pos[i]) < abs(out - pos[i]):
            out = out - 2*pi
        out_goal[i] = out
    return out_goal

class NomMpcClient:
    def __init__(self, srv):
        self.comoto_srv = srv
        self.ctrl_pub = rospy.Publisher("/jaco_adaptive_ctrl/goal", JointTrajectory, queue_size=1)
        
        self.joint_pos = None
        self.joint_pos_mut = Lock()

    def js_cb(self, state: JointState):
        self.joint_pos_mut.acquire()
        self.joint_pos = state.position[:7]
        self.joint_pos_mut.release()
        
    def nom_cb(self, evt):
        self.joint_pos_mut.acquire()
        if self.joint_pos is None:
            self.joint_pos_mut.release()
            return
        curr_posn = np.array(self.joint_pos)
        self.joint_pos_mut.release()

        n_plan_ts = 4
        goal = np.array([1.0656882781191814, 0.4604144797878419, -3.118373350993719, -1.2922323399336983, -0.1051192292831713, -1.3377377734798825, 1.806642277664842])
        goal = get_goal_nearest(goal, curr_posn)
        solve_req = SolveRequest()
        
        # add human traj
        for i in range(n_plan_ts):
            pt = JointTrajectoryPoint()
            pt.time_from_start = rospy.Duration(PLAN_TIME + i*PLAN_DT)
            pt.positions = list(np.tile([0, -1, 0.1], 11))
            solve_req.pred_traj.points.append(pt)
            
        # add start state (posn and vel)
        # TODO we could do better than this
        solve_req.joint_start = curr_posn
        solve_req.start_vel = [0]*7

        # add weights
        solve_req.weights = [0,0,0,0.7,0,0.]

        # add object set
        # TODO hardcode me to something else!
        solve_req.object_set = [0.752,-0.19,0.089, 0.752, 0.09, -0.089]

        # find nominal traj
        u_nom = 0.5*(goal - curr_posn)
        u_nom = np.clip(u_nom, -0.2, 0.2)
        nom_target = ((n_plan_ts - 1)*PLAN_DT*u_nom) + curr_posn
        nom_traj = np.linspace(curr_posn, nom_target, n_plan_ts)

        # add nominal traj
        for i in range(n_plan_ts):
            pt = JointTrajectoryPoint()
            pt.time_from_start = rospy.Duration(i*PLAN_DT)
            pt.positions = nom_traj[i,:]
            solve_req.nom_traj.points.append(pt)

        # print(solve_req)
        print("************\nResponse\n***************")
        try:
            resp: SolveResponse = self.comoto_srv(solve_req)
            print(resp.solve_traj)
            if resp.solve_traj.points[0].time_from_start.to_sec() < 0.15:
                resp.solve_traj.points = resp.solve_traj.points[1:]
            self.ctrl_pub.publish(resp.solve_traj)
        except rospy.ServiceException:
            print("Service exception!!")
            
        # print("==============================")
        
        
if __name__ == "__main__":
    rospy.init_node("test_nom", anonymous=True)
    comoto_srv = rospy.ServiceProxy("/comoto", Solve)
    nom_planner = NomMpcClient(comoto_srv)
    
    rospy.Subscriber("/right_arm/base_feedback/joint_state", JointState, nom_planner.js_cb)
    rospy.Timer(rospy.Duration(0.5), nom_planner.nom_cb)
    rospy.spin()