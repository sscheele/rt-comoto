#!/usr/bin/env python3
import baselines.rt_comoto
import baselines.rrt
import baselines.speed_adaptive
import baselines.nominal
import baselines.legible

from typing import List
import numpy as np
import rospy

from std_msgs.msg import Empty

class_dict = {
    "rrt": baselines.rrt.RRTBaseline,
    "comoto": baselines.rt_comoto.TrajectoryManager,
    "sa": baselines.speed_adaptive.VelocityController,
    "nominal": baselines.nominal.Nominal,
    'leg': baselines.legible.Legible
}

class TrajectoryRunner:
    def __init__(self):
        self.cancel_topic = rospy.Publisher('/jaco_adaptive_ctrl/stop', Empty)
    
    def run_trajectory(self, method: str, goal: List[float], timeout=0):
        """
        Runs a trajectory of type method. Assumes you have already spun up a ROS
        node!!

        method: one of 'rrt', 'comoto', 'sa', 'nominal'
        goal: ndarray or list of float
        """
        assert(len(goal) == 7)
        np_goal = np.asarray(goal)
        if method != 'leg':
            method_cls = class_dict[method](np_goal)
        else:
            try:
                method_cls = class_dict[method](np_goal)
            except ValueError:
                print('Legible not near a start, switching to nominal!')
                method_cls = class_dict['nominal'](np_goal)
        n_steps = 0
        while not method_cls.reached_goal:
            rospy.sleep(0.1)
            n_steps += 1
            if timeout > 0 and n_steps*0.1 >= timeout:
                method_cls.teardown()
                self.cancel_topic.publish(Empty())
                return False
        method_cls.teardown()
        self.cancel_topic.publish(Empty())
        return True   
