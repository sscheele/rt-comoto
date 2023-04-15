#!/usr/bin/env python3
import numpy as np
import rospy
import kinpy as kp

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from julia_comoto.msg import SolveRequest

PLAN_DT = 0.25

if __name__ == "__main__":
    rospy.init_node("test_nom", anonymous=True)
    req_pub = rospy.Publisher("/comoto", SolveRequest, queue_size=1)
    rospy.sleep(0.1)
    solve_req = SolveRequest()
    
    n_plan_ts = 4
    curr_posn = rospy.wait_for_message("/right_arm/base_feedback/joint_state", JointState)
    print("Got joint state!")
    curr_posn = np.array(curr_posn.position)[:7]
    goal = np.array([1.0656882781191814, 0.4604144797878419, -3.118373350993719, -1.2922323399336983, -0.1051192292831713, -1.3377377734798825, 1.806642277664842])
        
    # add human traj
    for i in range(n_plan_ts):
        pt = JointTrajectoryPoint()
        pt.time_from_start = rospy.Duration(i*PLAN_DT)
        pt.positions = list(np.tile([0, -1, 0.1], 11))
        solve_req.pred_traj.points.append(pt)
            
    # add start state (posn and vel)
    # TODO we could do better than this
    solve_req.joint_start = curr_posn
    solve_req.start_vel = [0]*7

    # add weights
    solve_req.weights = [0,0,0,10,0,0]

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

    req_pub.publish(solve_req)
    rospy.sleep(0.01)
    print("Published!")
    comoto_soln = rospy.wait_for_message("/jaco_adaptive_ctrl/goal", JointTrajectory)
    
    # load urdf and do fk
    chain = kp.build_chain_from_urdf(open('gen3.urdf').read())
    joint_names = chain.get_joint_parameter_names()
    def get_fk(vec):
        js_map = {joint_names[i]: vec[i] for i in range(len(vec))}
        fk_res = chain.forward_kinematics(js_map)
        return fk_res['EndEffector_Link'].pos
    
    nom_fk = []
    comoto_fk = []
    for i in range(nom_traj.shape[0]):
        nom_fk.append(get_fk(nom_traj[i,:]))
    for pt in comoto_soln.points:
        comoto_fk.append(get_fk(pt.positions))
    nom_fk = np.array(nom_fk)
    comoto_fk = np.array(comoto_fk)
    
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(nom_fk[:,0], nom_fk[:,1], nom_fk[:,2], marker='o')
    ax.scatter(comoto_fk[:,0], comoto_fk[:,1], comoto_fk[:,2], marker='^')
    ax.set_xlim(-0.5, 0.5)
    ax.set_xlabel("X")
    ax.set_ylim(-0.75, 0)
    ax.set_ylabel("Y")
    ax.set_zlim(0, 0.8)
    ax.set_zlabel("Z")
    plt.show()
