#!/usr/bin/env python3
import rospy
import tf2_ros
from sensor_msgs.msg import JointState

import urdfpy
import numpy as np

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
    x = rospy.wait_for_message(
            "/right_arm/base_feedback/joint_state", JointState)
    x = list(x.position)
    link_map = {f'Actuator{i}': x[i-1] for i in range(1,8)}
    fk_map = urdf.link_fk(cfg=link_map)
    for link, matr in fk_map.items():
        if link.name == 'Bracelet_Link':
            return matr[:-1,3]
    raise KeyError("No link named Bracelet_Link")
        
def get_camera_posn(buf):
    """
    Get current user hand position as  
    """
    posns = []
    for i in range(60):
        res = buf.lookup_transform('tracker/user_1/left_hand', 
            'tracker_depth_frame', rospy.Time())
        posns.append([res.transform.translation.x, res.transform.translation.y, res.transform.translation.z])
        rospy.sleep(0.05)
    return np.array(posns).mean(axis=0)
    
    
def do_transform(A, posns):
    """
    A: (4,4)
    posns: (n, 3)    
    """
    n = posns.shape[0]
    x = np.hstack((posns, np.ones((n, 1)))).T
    trans = np.matmul(A, x)
    return trans[:-1,:].T

if __name__ == "__main__":
    rospy.init_node("calibration", anonymous=True)
    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)
    
    urdf = urdfpy.URDF.load('gen3.urdf')
    
    camera_points = []
    arm_points = []
    
    while True:
        msg = input('Enter to register a point, or "exit" to estimate')    
        if msg.strip() == "exit":
            break
        rospy.sleep(3)
        camera_points.append(get_camera_posn(tf_buf))
        print("Registered camera point: ", camera_points[-1])
        arm_points.append(get_jaco_fk(urdf))
        print("Registered arm point: ", arm_points[-1])
    camera_points, arm_points = np.asarray(camera_points), np.asarray(arm_points)
    A = solve_camera_matr(camera_points, arm_points)
    print(repr(A))
    
    est = do_transform(A, camera_points)
    errs = np.linalg.norm(est - arm_points, axis=1)
    print("Mean error: ", errs.mean())
    print("Std: ", errs.std())