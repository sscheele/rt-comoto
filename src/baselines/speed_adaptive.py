#!/usr/bin/env python3
import rospy
import os.path

from sensor_msgs.msg import JointState
from kortex_driver.msg import Base_JointSpeeds, JointSpeed

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from scipy.spatial import distance_matrix
import sys
#import urdfpy
import kinpy as kp
from threading import Lock


MAX_SPEED = 0.2
urdf_path = os.path.join(os.path.dirname(__file__), 'gen3.urdf')
urdf = kp.build_chain_from_urdf(open(urdf_path).read())
TOLERANCE = 0.1
dict = {"HalfArm1_Link": 0, "HalfArm2_Link": 1, "ForeArm_Link": 2, "SphericalWrist1_Link": 3, "SphericalWrist2_Link": 4, "Bracelet_Link": 5, "EndEffector_Link": 6}
P_GAIN = 0.7

BETA = 0.1
GAMMA = -0.9
D_SLOW = 0.25
D_STOP = 0.1

def js_sub(vec1, vec2):
    diff = vec1 - vec2
    diff[diff > np.pi] -= 2*np.pi
    diff[diff < -np.pi] += 2*np.pi
    return diff

class VelocityController:
    def __init__(self, goal):
        self.mutex = Lock()
        goal = np.asarray(goal)
        posn = rospy.wait_for_message('/right_arm/base_feedback/joint_state', 
            JointState).position[:7]
        
        self.goal = posn + js_sub(goal, posn)
        print(self.goal)
        self.position = np.zeros(shape=(7,3))
        self.humanPosition = np.zeros(shape=(21,3))
        self.distanceMatrix = np.zeros(shape=(21,7))
        self.error = []
        self.scalar = 0
        self.distance = 0
        self.factor = 0.99
        self.vel = np.array([0, 0, 0, 0, 0, 0, 0])
        self.dir = np.array([0, 0, 0, 0, 0, 0, 0])
        self.reached_goal = False
        self.vel_pub = rospy.Publisher(f"/right_arm/in/joint_velocity",
        Base_JointSpeeds, queue_size=1)

        self.is_torn_down = False
        
        self.pub_subs = [
            rospy.Subscriber(f"/right_arm/base_feedback/joint_state", JointState, self.robot_joint_state_cb),
            rospy.Subscriber(f"/human_posn", JointTrajectoryPoint, self.human_joint_state_cb),
            self.vel_pub
        ]

        self.timers = [
            rospy.Timer(rospy.Duration(1.0/10.0), self.sendvel)
        ]

    def teardown(self):
        if self.is_torn_down:
            return
        self.is_torn_down = True
        for x in self.pub_subs:
            x.unregister()
        for x in self.timers:
            x.shutdown()

    def human_joint_state_cb(self, x: JointTrajectoryPoint):
        self.mutex.acquire()
        self.humanPosition = np.asarray(x.positions)
        self.mutex.release()
       
    def robot_joint_state_cb(self, x):
        self.mutex.acquire()
        tempPosition = np.array(x.position[:7])
        self.error = js_sub(self.goal, tempPosition)
        if np.any(self.humanPosition):
            self.distance = self.findDistance(tempPosition)
            self.scalar = self.findScalar()
            # print("Factor: ", self.scalar)
        else:
            self.mutex.release()
            return
        if (max(abs(self.error)) <= TOLERANCE):
            self.reached_goal = True
            self.vel = [0, 0, 0, 0, 0, 0, 0]
        else:
            #self.vel = self.dir * self.factor * np.clip(np.abs(self.scalar), 0, 1)
            self.vel = P_GAIN * self.error
                        
            if (max(abs(self.vel)) > MAX_SPEED):
                self.vel /= max(abs(self.vel))
                self.vel *= MAX_SPEED
            
            # if self.scalar < 0.1:
            #     self.vel = np.zeros(7)
            # else:
            #     self.vel *= self.scalar
            self.vel *= self.scalar
        self.mutex.release()


    def findDistance(self, tempPosition):
        link_map = {f'Actuator{i}': tempPosition[i-1] for i in range(1,8)}
        fk_map = urdf.forward_kinematics(link_map)
        for link, matr in fk_map.items():
            if link in dict:
                self.position[dict.get(link)] = np.asarray(matr.pos)
        matrix = distance_matrix(self.humanPosition.reshape(21,3), self.position)
        return np.min(matrix)
   
    def findScalar(self):
        # print("Distance: ", self.distance)
        if self.distance < D_STOP:
            return 0
        if self.distance > D_SLOW:
            return 1
        ret = 1.2 - BETA * (self.distance - D_STOP + 0.05) ** GAMMA
        return np.clip(ret, 0, 1)
        
    def sendvel(self, event=None):
        self.mutex.acquire()
        msg = Base_JointSpeeds()
        for i in range(len(self.vel)):
            tmp = JointSpeed()
            tmp.joint_identifier = i
            # if self.reached_goal:
            #     tmp.value = 0
            tmp.value = self.vel[i]
            tmp.duration = 1
            msg.joint_speeds.append(tmp)
        msg.duration = 1
        self.vel_pub.publish(msg)
        self.mutex.release()
        if self.reached_goal:
            self.teardown()
   

if __name__ == "__main__":
    from baselines.positions import posns
    rospy.init_node("vel_control", anonymous=True)
    velocityController = VelocityController(posns[0])
    rospy.spin()