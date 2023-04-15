#!/usr/bin/env python3
import rospy
from baselines import positions, TrajectoryRunner
from record_data import DataRecorder

if __name__ == "__main__":
    rospy.init_node("rcd-tester", anonymous=True)
    rcd = DataRecorder()
    runner = TrajectoryRunner()
    runner.run_trajectory('comoto', positions.test_2)
    rcd.stop()
