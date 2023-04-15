import rospy
from sensor_msgs.msg import JointState

if __name__ == "__main__":
    rospy.init_node("js_printer", anonymous=True)
    while True:
        input()
        x = rospy.wait_for_message(
            "/right_arm/base_feedback/joint_state", JointState)
        print(list(x.position), ",")
    