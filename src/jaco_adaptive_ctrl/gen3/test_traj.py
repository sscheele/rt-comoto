#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

traj = [
    [-1.3101399677356094, -0.5496966890166393, 0.43562381228008046, -1.6604718849166984, 0.841452910653654, 0.9221727906617377, 0.7680274954503247, 0.0035087795780417252],
    [-2.282937852823099, -0.5604265650485685, 0.24525751222801534, -1.5948404103805673, -0.9510792737656759, 0.7309108861798358, 2.2661450242534955, 0.0035087795780417252],
    [-2.2719949239039634, -0.3859128146089841, 0.24537505750062868, -0.7335512106635678, -2.270855623589628, 0.6596380375065902, -2.3833416890801766, 0.0035087795780417252],
    [-1.1240393360253078, -0.32535998827163404, 0.3038349554715555, -1.0075579962674226, -1.4626549423067035, -0.7619495626315187, 2.840550255547781, 0.0035087795780417252]
]

def make_traj(arr, time_per_point=2, base_time=0):
    msg = JointTrajectory()
    for i, item in enumerate(arr):
        tmp = JointTrajectoryPoint()
        tmp.positions = item
        tmp.time_from_start = rospy.Duration(base_time + i*time_per_point)
        msg.points.append(tmp)
    return msg, tmp.time_from_start.to_sec()

if __name__ == "__main__":
    rospy.init_node("traj_test", anonymous=True)
    pub = rospy.Publisher("/jaco_adaptive_ctrl/goal", JointTrajectory, queue_size=1)
    rospy.sleep(1)
    
    traj = [x[:-1] for x in traj]
    
    msg, t = make_traj(traj[:1], base_time=5)
    pub.publish(msg)
    rospy.sleep(1.0)
    msg, t = make_traj(traj[1:3], base_time=t+3)
    pub.publish(msg)
    rospy.sleep(1.0)
    msg, t = make_traj(traj[3:], base_time=t+2)
    pub.publish(msg)