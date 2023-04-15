import rospy
from kortex_driver.msg import Base_JointSpeeds, JointSpeed
from time import sleep


if __name__ == "__main__":
    rospy.init_node("test_interrupt", anonymous=True)
    vel_pub = rospy.Publisher("/right_arm/in/joint_velocity", Base_JointSpeeds, queue_size=1)
    
    rospy.sleep(2.0)
    msg = Base_JointSpeeds()
    tmp = JointSpeed()
    tmp.joint_identifier = 0
    tmp.value = 0.15
    tmp.duration = 1
    msg.joint_speeds.append(tmp)
    msg.duration = 1
    vel_pub.publish(msg)
    
    # rospy.sleep(2.0)
    # msg = Base_JointSpeeds()
    # tmp = JointSpeed()
    # tmp.joint_identifier = 0
    # tmp.value = 0.15
    # tmp.duration = 1
    # msg.joint_speeds.append(tmp)
    # msg.duration = 1
    # vel_pub.publish(msg)
    
    rospy.sleep(2.0)
    msg = Base_JointSpeeds()
    tmp = JointSpeed()
    tmp.joint_identifier = 0
    tmp.value = 0.0
    tmp.duration = 1
    msg.joint_speeds.append(tmp)
    msg.duration = 1
    vel_pub.publish(msg)
    
