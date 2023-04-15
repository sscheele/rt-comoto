import numpy as np
import rospy
import sys

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal 

JOINT_TARGET = [-1.3055470801196583, -0.9100197213432706, 0.3673008472777747, -0.8728350683069293, -0.31091180672573593, -1.2278695953408816, 1.4791600163205234]

traj_states =  [(-1.9113778785929876, -0.8093069583997439, -0.21720315744911822, -1.011911199385266, 0.22587033199299736, -1.0345086538668058, 1.4452068429508853), (-1.8735137576918135, -0.8156014526506793, -0.18067193189158526, -1.0032190185031182, 0.19232168156704008, -1.0465936364606359, 1.4473289013247788), (-1.8261828747736981, -0.8234696801918301, -0.1350071858582941, -0.9923535864939474, 0.15038519891948926, -1.0617000946305937, 1.4499815163840253), (-1.8133546638488993, -0.825602230445681, -0.1226305617137713, -0.9894087084367672, 0.13901906594171187, -1.0657944299692392, 1.4507004603696931), (-1.7816170415275103, -0.8308783604783858, -0.0920099705292454, -0.9821228303459162, 0.11089829968000768, -1.0759241598551723, 1.4524791856648118), (-1.741944455736917, -0.8374735000897318, -0.053733947711995834, -0.9730154784817737, 0.07574731253601907, -1.0885863317120328, 1.4547025969742562), (-1.732920826708238, -0.8389735784683954, -0.04502797207546692, -0.9709439891372287, 0.06775213490562108, -1.0914663735868317, 1.4552083173074164), (-1.7062100398837736, -0.843413924514695, -0.01925753752914691, -0.9648122169169902, 0.04408578653303642, -1.099991520571772, 1.4567052895949157), (-1.6744943381430701, -0.8486862818010689, 0.011341617523206428, -0.9575315071467424, 0.01598496725989837, -1.110114063192094, 1.4584827563359337), (-1.6517507227575334, -0.8526518670323624, 0.03354684078241675, -0.9522532815191531, -0.004620130388887592, -1.117530751202637, 1.4597843726728725), (-1.6403636095960716, -0.8545386667197854, 0.04452433983141334, -0.9496411345696661, -0.014694258455160873, -1.121159865879422, 1.4604216509395562), (-1.6349614996742645, -0.8574622505377585, 0.05182666441129874, -0.9480484974588479, -0.023036600577413142, -1.1241720484979174, 1.4609351308468783), (-1.6421940660361019, -0.8678850495569502, 0.05469298044635752, -0.9490789618470443, -0.03304865879667025, -1.1277206020945334, 1.4615073772502347), (-1.615148881404948, -0.8730314362386156, 0.08089576085713163, -0.9430979302636585, -0.057316655535870696, -1.1365148447394262, 1.4630390477165363), (-1.6173385421666948, -0.8755461056838179, 0.08300254997970014, -0.942337205247015, -0.06274455970117107, -1.1384392843483626, 1.4633485263983341), (-1.6200794080225658, -0.8757586694608687, 0.08424144285070273, -0.9410856716176721, -0.0678721954866261, -1.1404285148243658, 1.4636209283841006), (-1.6329144587174107, -0.8788864159909069, 0.07805450791375607, -0.9425677816127285, -0.06732903000442836, -1.1402008001715513, 1.4635339937399394), (-1.6235709941299918, -0.8853618895486828, 0.091986830654679, -0.9395770568167586, -0.08427035347134475, -1.1464202299995736, 1.4645610160573634), (-1.583817904446777, -0.8884442853294862, 0.12640130596799698, -0.9312342796282149, -0.11260087928780547, -1.1566015449224578, 1.466385910884926)]
traj_times =  [485589673042, 485839673042, 486089673042, 486138952016, 486388952016, 486638952016, 486680306911, 486930306911, 487167781829, 487417781829, 487516455888, 487688370943, 487938370943, 488187595844, 488383416891, 488573447942, 488823447942, 489073447942, 489323447942]

# give 2 seconds to reach start
traj_states = [traj_states[0]] + traj_states
traj_times = np.array(traj_times) - traj_times[0] 
traj_times = np.concatenate(([0], traj_times + 2))

def go_to_point(pub, point):
    out_goal = FollowJointTrajectoryActionGoal()    
    out_traj = JointTrajectory()
    # TODO use self.joint_names
    # out_traj.joint_names = [f"Actuator{i}" for i in range(1,8)]
    out_traj.joint_names = [f"joint_{i}" for i in range(1,8)]
    traj_point = JointTrajectoryPoint()
    traj_point.time_from_start = rospy.Duration(secs=3)
    traj_point.positions = list(point)
    out_traj.points.append(traj_point)
    out_goal.goal.trajectory = out_traj
    rospy.sleep(1)
    pub.publish(out_goal)

if __name__ == "__main__":
    rospy.init_node("do_traj", anonymous=True)
    traj_pub = rospy.Publisher("/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=1)
    
    input("Enter to move to start...")
    go_to_point(traj_pub, traj_states[0])
    input("Enter to start moving...")
    
    if len(sys.argv) > 1 and sys.argv[1] == 'nom':
        go_to_point(traj_pub, JOINT_TARGET)
    else:
        out_goal = FollowJointTrajectoryActionGoal()    
        out_traj = JointTrajectory()
        # TODO use self.joint_names
        # out_traj.joint_names = [f"Actuator{i}" for i in range(1,8)]
        out_traj.joint_names = [f"joint_{i}" for i in range(1,8)]
        for idx, pt in enumerate(traj_states):
            traj_point = JointTrajectoryPoint()
            traj_point.time_from_start = rospy.Duration(nsecs=traj_times[idx])
            traj_point.positions = list(pt)
            out_traj.points.append(traj_point)
        out_goal.goal.trajectory = out_traj
        rospy.sleep(1)
        traj_pub.publish(out_goal)