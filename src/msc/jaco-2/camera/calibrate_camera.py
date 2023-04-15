#!/usr/bin/env python3

import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal

import urdfpy
import numpy as np

EE_LINK_NAME = "j2s7s300_ee_base"
JOINT_POSNS = [
    [2.195020247336858, 2.4359393869253503, 3.4537441618029616, 1.4788376406705697, 6.096018345768817, 3.8426000357325925, 5.034488726849849],
    [1.9996231860513631, 1.9275969291057342, 3.4083708210468457, 2.3743074477143007, 6.0960305963098325, 4.123099874390169, 5.034511097403006],
    [1.532458791322661, 2.4224267738703973, 3.4023456853966505, 1.8796017062564352, 6.094488626038676, 4.282899127174848, 5.036010989728939],
    [1.0354992166342611, 2.022691753729738, 3.350226024967146, 2.320199204898834, 6.096023139458779, 4.297959835771464, 5.034511097403006],
    [1.598291733995256, 2.624044579988154, 3.1477456209723123, 0.8739499341182643, 6.094493952360856, 3.454494906914255, 5.0345063037130435],
    [2.093943962904536, 2.3054282482288184, 3.1493597629090093, 1.5639144522220227, 0.07210887319607033, 3.8441446691648387, 5.034479139469925]
]

JOINT_STATE_TOPIC = "/j2s7s300_driver/out/joint_state"

def solve_camera_matr(camera_posns, arm_posns):
    """
    Inputs:
    camera_posns (n, 3) matrix of positions in camera frame
    arm_posns (n, 3) matrix of positions in arm frame
    
    Returns:
    A (4, 4) transformation matrix from camera frame to arm frame
    """
    camera_posns = np.asarray(camera_posns)
    arm_posns = np.asarray(arm_posns)
    
    n = camera_posns.shape[0]
    x = np.hstack((camera_posns, np.ones((n, 1)))).T
    y = np.hstack((arm_posns, np.ones((n, 1)))).T
    
    # Solve Ax = y
    A = np.linalg.pinv(x.T)@y.T
    return A.T
    
def get_jaco_fk(urdf):
    """
    Reads current jaco joint state and gets the link 7 fk
    """
    x = rospy.wait_for_message(JOINT_STATE_TOPIC, JointState)
    x = list(x.position)
    link_map = {f'j2s7s300_joint_{i}': x[i-1] for i in range(1,8)}
    fk_map = urdf.link_fk(cfg=link_map)
    for link, matr in fk_map.items():
        if link.name == EE_LINK_NAME:
            return matr[:-1,3]
    raise KeyError("No link named " + EE_LINK_NAME)
        
def get_camera_posn(buf):
    """
    Get current user hand position as  
    """
    posns_arr = []
    for i in range(80):
        res = buf.lookup_transform('tracker/user_1/left_hand', 
            'tracker_depth_frame', rospy.Time())
        posns_arr.append([res.transform.translation.x, res.transform.translation.y, 
            res.transform.translation.z])
        rospy.sleep(0.05)
    return np.mean(np.array(posns_arr), axis=0)    
    
    
def do_transform(A, posns):
    """
    A: (4,4)
    posns: (n, 3)    
    """
    n = posns.shape[0]
    x = np.hstack((posns, np.ones((n, 1)))).T
    trans = np.matmul(A, x)
    return trans[:-1,:].T

def go_to_point(point, traj_pub):
    """
    Drives robot to point point, and sleeps until completion
    """
    goal = FollowJointTrajectoryActionGoal()
    trajectory = JointTrajectory()
    trajectory.joint_names = [f"j2s7s300_joint_{i}" for i in range(1,8)]
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.time_from_start = rospy.Duration(secs=3)
    trajectory_point.positions = point
    trajectory.points.append(trajectory_point)
    goal.goal.trajectory = trajectory
    traj_pub.publish(goal)
    rospy.sleep(3)

if __name__ == "__main__":
    rospy.init_node("calibration", anonymous=True)
    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)

    traj_pub = rospy.Publisher(f"/jaco_trajectory_controller/follow_joint_trajectory/goal", 
        FollowJointTrajectoryActionGoal, queue_size=1)
    
    urdf = urdfpy.URDF.load('jaco.urdf')
    
    camera_points = []
    arm_points = []
    
    for pt in JOINT_POSNS:
        input("Enter to drive to next point")
        go_to_point(pt, traj_pub)
        
        input("Enter to register arm position (3 sec delay)")
        rospy.sleep(3)
        camera_points.append(get_camera_posn(tf_buf))
        arm_points.append(get_jaco_fk(urdf))

        print("*******")
        print("Registered arm point: ", arm_points[-1])
        print("Registered camera point: ", camera_points[-1])
        print("*******")
        
    camera_points, arm_points = np.asarray(camera_points), np.asarray(arm_points)
    A = solve_camera_matr(camera_points, arm_points)
    print(A)
    
    est = do_transform(A, camera_points)
    errs = np.linalg.norm(est - arm_points, axis=1)
    print("Mean error: ", errs.mean())
    print("Std: ", errs.std())