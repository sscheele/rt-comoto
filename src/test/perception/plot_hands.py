#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from threading import Lock
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

RH_IDX = 20

class PredictionPlotter:
    def __init__(self, fig, ax):
        self.fig = fig
        self.ax = ax

        self.rh_posn = None
        self.rh_posn_mut = Lock()
    
    def posn_cb(self, posn: JointTrajectoryPoint):
        """
        Handle position (actually a trajectory from ~2s ago to now)
        by saving the most recent position to self.rh_posn 
        """
        self.rh_posn_mut.acquire()
        latest_pt = np.array(posn.positions)
        latest_pt = latest_pt.reshape((21, 3))
        self.rh_posn = latest_pt[RH_IDX, :]
        self.lh_posn = latest_pt[16, :]
        self.rh_posn_mut.release()

    def pred_cb(self, traj: JointTrajectory):
        """
        Handle prediction by saving to self.rh_pred 
        Also repaint graph
        """

        # set pred_traj to be a (ts, 3) matrix of rh points
        pred_traj = np.array(
            [x.positions for x in traj.points]
        )
        pred_traj = pred_traj.reshape((pred_traj.shape[0], 21, 3))
        pred_traj = pred_traj[:,RH_IDX,:]

        # plot
        self.ax.cla()
        self.rh_posn_mut.acquire()
        if self.rh_posn is None:
            self.rh_posn_mut.release()
            return
            
        self.ax.scatter([self.rh_posn[0]], [self.rh_posn[1]], [self.rh_posn[2]], marker='^')
        # self.ax.scatter(pred_traj[:,0], pred_traj[:,1], pred_traj[:,2], marker='o')
        self.ax.scatter([self.lh_posn[0]], [self.lh_posn[1]], [self.lh_posn[2]], marker='o')
        
        self.rh_posn_mut.release()

        # draw
        self.ax.set_xlim(-1, 1)
        self.ax.set_xlabel("X")
        self.ax.set_ylim(-2, 0)
        self.ax.set_ylabel("Y")
        self.ax.set_zlim(-0.2, 2.5)
        self.ax.set_zlabel("Z")
        # self.fig.canvas.draw()
        # self.ax.draw()
        plt.draw()
        self.fig.canvas.flush_events()

if __name__ == "__main__":
    rospy.init_node("pred_viz", anonymous=True)
    f = plt.figure()
    ax = f.add_subplot(projection='3d')
    plotter = PredictionPlotter(f, ax)
    
    rospy.Subscriber("/human_posn", JointTrajectoryPoint, plotter.posn_cb, queue_size=1)
    rospy.Subscriber("/prediction_traj", JointTrajectory, plotter.pred_cb, queue_size=1)

    plt.show()
    rospy.spin()
    