import rospy
from control_msgs.msg import GripperCommandActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


# position: 0.6, effort: 0.01
if __name__ == "__main__":
    rospy.init_node("setup_camera_arm", anonymous=True)
    gripper_pub = rospy.Publisher("/left_arm/left_arm_robotiq_2f_85_gripper_controller/gripper_cmd/goal", GripperCommandActionGoal)
    joint_pub = rospy.Publisher("/left_arm/jaco_adaptive_ctrl/goal", JointTrajectory)
    rospy.sleep(1.0)
    
    input("Enter to move to start...")
    
    joint_msg = JointTrajectory()
    goal_pt = JointTrajectoryPoint()
    goal_pt.positions = [3.0061298994784766, 1.001249899751406, -1.7798582007916428, -0.913663990978935, 2.1693921155348077, -1.464624083616708, 0.3127689288223898]
    goal_pt.time_from_start = rospy.Duration(6.0)
    joint_msg.points.append(goal_pt)
    joint_pub.publish(joint_msg)
    
    input("Enter to close gripper...")
    
    gripper_msg = GripperCommandActionGoal()
    gripper_msg.goal.command.position = 0.65
    gripper_msg.goal.command.max_effort = 0.01
    gripper_pub.publish(gripper_msg)