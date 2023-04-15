#!/usr/bin/env python3
import rospy
import kinpy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from julia_comoto.msg import SolveRequest

URDF_PATH = "gen3.urdf"
EE_LINK = "EndEffector_Link"
class NomTrajPlot:
    def __init__(self, fig, ax):
        self.fig = fig
        self.ax = ax
        
        self.colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
        self.color_idx = 0
        
        self.chain = kinpy.build_chain_from_urdf(open(URDF_PATH).read())
        
    def nom_cb(self, req: SolveRequest):
        posns = [x.positions for x in req.nom_traj.points]
        joint_names = self.chain.get_joint_parameter_names()
        
        cart_posns = []
        for posn in posns:
            angle_map = {joint_names[i]: posn[i] for i in range(len(posn))}
            cart_posn = self.chain.forward_kinematics(angle_map)[EE_LINK].pos
            cart_posns.append(cart_posn)

        cart_posns = np.asarray(cart_posns)        
        self.ax.scatter(cart_posns[0,0], cart_posns[0,1], cart_posns[0,2], 
            color='r')
        self.ax.scatter(cart_posns[1,0], cart_posns[1,1], cart_posns[1,2], 
            color='g')
        self.color_idx = (self.color_idx + 1)%len(self.colors)
          
        self.ax.set_xlim(-0.5, 0.5)
        self.ax.set_xlabel("X")
        self.ax.set_ylim(-0.75, 0)
        self.ax.set_ylabel("Y")
        self.ax.set_zlim(0, 0.8)
        self.ax.set_zlabel("Z")
        
        plt.draw()
        self.fig.canvas.flush_events() 

if __name__ == "__main__":
    rospy.init_node("nom_plot", anonymous=True)
    f = plt.figure()
    ax = f.add_subplot(projection='3d')
    plotter = NomTrajPlot(f, ax)
    
    rospy.Subscriber("/comoto", SolveRequest, plotter.nom_cb, queue_size=1)
    plt.show()
    rospy.spin()