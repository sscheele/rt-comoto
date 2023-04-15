#!/usr/bin/env python
import rospy
import os.path
from cv_bridge import CvBridge
import cv2
import pickle

from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Empty

CAMERA_RATE = 15 # camera rate, Hz

SCRIPT_DIR = os.path.dirname(__file__)
SUBJ_DIR = os.path.join(SCRIPT_DIR, "data", "subjects")
FRAMES_DIR = os.path.join(SCRIPT_DIR, 'tmp_frames')

def infer_subj_id(restore):
    n_subjects = len(os.listdir(SUBJ_DIR))
    id = n_subjects
    if restore:
        id -= 1
    return "%03d" % id

class DataRecorder:
    def __init__(self, id=None, restore=False):
        # setup ID and data dir
        self.id = id
        if self.id is None:
            self.id = infer_subj_id(restore)
        self.data_dir = os.path.join(SUBJ_DIR, self.id)

        if not restore:
            os.mkdir(self.data_dir)
            # clear frames dir
            for fname in os.listdir(FRAMES_DIR):
                os.unlink(os.path.join(FRAMES_DIR, fname))

        
        self.n_frames = len(os.listdir(FRAMES_DIR))
        
        self.skel_points = []
        self.last_skel_pt = None
        
        self.joint_points = []
        self.last_joint_pt = None
        
        self.cvb = CvBridge()
        
        self.pub_subs = [
            rospy.Subscriber('/camera/rgb/image', Image, self.img_cb, queue_size=1),
            rospy.Subscriber('/human_posn', JointTrajectoryPoint, self.skel_cb, queue_size=1),
            rospy.Subscriber('/right_arm/base_feedback/joint_state', JointState, self.js_cb),
            rospy.Subscriber('/recorder/stop', Empty, self.stop_cb)
        ]
        
    def stop(self):
        self.stop_cb(None)
                
    def stop_cb(self, _: Empty):
        for x in self.pub_subs:
            x.unregister()

        skel_pkl_out = os.path.join(self.data_dir, f"{self.id}-skel.pkl")
        with open(skel_pkl_out, 'wb') as f:
            pickle.dump(self.skel_points, f)
        
        joint_pkl_out = os.path.join(self.data_dir, f"{self.id}-joints.pkl")
        with open(joint_pkl_out, 'wb') as f:
            pickle.dump(self.joint_points, f)
        
        os.system(f"ffmpeg -framerate {CAMERA_RATE} -pattern_type glob -i '{FRAMES_DIR}/*.png' \
        -c:v libx264 -pix_fmt yuv420p {self.data_dir}/{self.id}.mp4")

    def img_cb(self, msg: Image):
        img = self.cvb.imgmsg_to_cv2(msg)
        img_name = os.path.join(FRAMES_DIR, "%08d.png" % self.n_frames)
        self.n_frames += 1
        cv2.imwrite(img_name, img)
        
        self.skel_points.append(self.last_skel_pt)
        self.joint_points.append(self.last_joint_pt)
    
    def js_cb(self, x: JointState):
        self.last_joint_pt = x.position[:7]
        
    def skel_cb(self, pt: JointTrajectoryPoint):
        self.last_skel_pt = pt.positions
    
if __name__ == "__main__":
    rospy.init_node("recorder", anonymous=True)
    x = DataRecorder()
    rospy.spin()