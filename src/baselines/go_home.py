#!/usr/bin/env python3
import rospy
import numpy as np

from baselines import TrajectoryRunner, positions
    
if __name__ == "__main__":
    rospy.init_node("baseline", anonymous=True)
    goal = positions.home
    runner = TrajectoryRunner()
    runner.run_trajectory('nominal', goal)
    print("Finished!")
    rospy.signal_shutdown("Done..")
    exit(0)