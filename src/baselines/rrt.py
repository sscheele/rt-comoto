#!/usr/bin/env python3
import rospy
import numpy as np
import kinpy as kp
import os.path
import sys
from baselines import positions

from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

GOAL_THRESH = 0.05

RRT_MAX_LEN = 1.5 # this is a JS distance
RRT_COLLISION_DIST = 0.1 # give 10 cm cartesian distance from the human

# position constraints: 
# y no greater than 0.02 (avoid post collision)
# z no less than 0.3 (avoid table collision)
VALID_MIN = np.array([-100, -100, 0.31])
VALID_MAX = np.array([100, 0.02, 100])

# joint limits (approx)
JOINT_LIMITS = (
    # np.array([0.61, 0, 2, -1.74, -np.pi, -np.pi, -np.pi]),
    # np.array([2.3, 0.74, 4.2, 0.42, np.pi, np.pi, np.pi])
    -np.pi*np.ones(7),
    np.pi*np.ones(7)
)

urdf_path = os.path.join(os.path.dirname(__file__), 'gen3.urdf')
gen3_chain = kp.build_chain_from_urdf(open(urdf_path).read())
gen3_joints = gen3_chain.get_joint_parameter_names()
jaco_links = ["HalfArm2_Link",
    "ForeArm_Link",
    "SphericalWrist1_Link",
    "SphericalWrist2_Link",
    "Bracelet_Link",
    "EndEffector_Link"
]

def rand_between(low, high):
    """
    Generate a series of random numbers between low and high 
    """
    return low + (high-low)*np.random.random(low.shape[0])

def rand_posn():
    """
    Generate a random position that's more likely to be
    valid given constraints 
    """
    posn = rand_between(JOINT_LIMITS[0], JOINT_LIMITS[1])
    posn[posn > np.pi] -= 2*np.pi
    posn[posn < -np.pi] += 2*np.pi
    return posn

def jaco_fk(posn):
    """
    Posn: length-7 vector
    Returns fk results as an (n_links, 3) ndarray 
    """
    fk_map = gen3_chain.forward_kinematics(
        {gen3_joints[i]: posn[i] for i in range(len(posn))})
    fk_arr = []
    for link_name in jaco_links:
        fk_arr.append(fk_map[link_name].pos)
    return np.array(fk_arr)

def is_traj_valid(soln, pred):
    """
    soln: a (n_points, 7) matrix
    pred: a (ts, JOINTS, 3) matrix
    """
    for i in range(soln.shape[0]-1):
        if not is_segment_valid((soln[i,:], soln[i+1,:]), pred, i==0):
            return False
    return True
    
def is_segment_valid(ends, pred, exclude_zero=False):
    """
    Checks to ensure both ends of a segment are within-bounds and that
    the segment doesn't collide with pred
    
    ends: (start, end) tuple
    pred: (ts, JOINTS, 3) shape ndarray
    exclude_zero: if true, excludes the first position from consideration
        (useful for not getting stuck when the 0 index narrowly violates)
    """
    pred = pred.reshape(-1, 3)
    n_interp = max(3, int(16*np.linalg.norm(ends[1] - ends[0])/RRT_MAX_LEN))
    interp_posns = np.linspace(ends[0], ends[1], n_interp)
    if exclude_zero:
        interp_posns = interp_posns[1:, :]
    fk_posns = np.array([jaco_fk(x) for x in interp_posns]).reshape((-1, 3))
    if np.any(fk_posns < VALID_MIN) or np.any(fk_posns > VALID_MAX):
        # print("Segment invalid (oob): ")
        # invalid_mask = np.logical_or(np.any(fk_posns < VALID_MIN, axis=1), 
        #     np.any(fk_posns > VALID_MAX, axis=1))
        # print(fk_posns[invalid_mask])
        return False
    for posn in fk_posns:
        if np.any(np.linalg.norm(pred - posn, axis=1) < RRT_COLLISION_DIST):
            # print("Segment invalid (collision)")
            return False
    return True
    
def shortcut_soln(soln, pred, dt=6.8):
    """
    Do shortcutting by connecting increasingly distant parts of a solution
    
    soln: (n_points, 7)
    pred: (ts, JOINTS, 3)
    dt: initial timestep (used to maintain similar speed throughout)
    """
    soln = np.asarray(soln)
    return soln, dt*(1+np.arange(soln.shape[0]))
    end_idx = soln.shape[0] - 1
    out_soln = soln.copy()
    dt_arr = [dt]
    while end_idx > 1:
        n_shortcut = 2
        # while we can still shortcut and previous shortcut valid
        while end_idx - n_shortcut >= 0 and \
            is_segment_valid((out_soln[end_idx-n_shortcut,:], out_soln[end_idx,:]), pred):
            n_shortcut += 1
        n_shortcut -= 1 # undo breaking shortcut
        dt_arr.append(dt*n_shortcut)
        # perform shortcut
        out_soln = np.concatenate((out_soln[:end_idx-n_shortcut+1],
            out_soln[end_idx:]), axis=0)
        # end_idx set to idx of shortcut start
        end_idx = end_idx - n_shortcut
    dt_arr = dt_arr[::-1] # reverse
    dt_arr = np.cumsum(dt_arr)
    return out_soln, dt_arr

    
class RRTBaseline:
    def __init__(self, goal):
        self.ctrl_pub = rospy.Publisher("/jaco_adaptive_ctrl/goal", JointTrajectory, queue_size=1)
        self.stop_pub = rospy.Publisher("/jaco_adaptive_ctrl/stop", Empty, queue_size=1)

        self.goal = np.asarray(goal)
        self.cart_goal = jaco_fk(self.goal)[-1] 
        print("Cartesian goal: ", self.cart_goal)
        self.reached_goal = False

        self.curr_pred = None
        self.soln = None
        self.curr_posn = None

        self.is_torn_down = False
        
        self.pub_subs = [
            rospy.Subscriber("/right_arm/base_feedback/joint_state", JointState, self.state_cb),
            rospy.Subscriber("/prediction_traj", JointTrajectory, self.check_soln_cb),
            self.ctrl_pub,
            self.stop_pub
        ]
        
    def teardown(self):
        if self.is_torn_down:
            return
        self.is_torn_down = True
        for x in self.pub_subs:
            try:
                x.unregister()
            except AssertionError:
                pass
        
    def state_cb(self, state: JointState):
        self.curr_posn = np.array(state.position)[:7]
        cart_posn = jaco_fk(self.curr_posn)[-1]
        if np.linalg.norm(cart_posn - self.cart_goal) < GOAL_THRESH:
            print("Reached goal!")
            self.reached_goal = True
            self.teardown()
        
    def check_soln_cb(self, pred: JointTrajectory):
        next_pred = np.array([x.positions for x in pred.points])
        next_pred = next_pred.reshape((next_pred.shape[0], -1, 3))
        self.curr_pred = next_pred
        if self.curr_posn is None:
            return
        curr_posn = np.copy(self.curr_posn)
        
        if self.soln is None:
            self.replan()
            return
            
        soln = np.concatenate((curr_posn.reshape((1, -1)), self.soln), axis=0)
        # soln = self.soln.copy()
        if not is_traj_valid(soln, next_pred):
            self.stop_pub.publish(Empty())
            self.soln = None
            self.replan()
    
    def replan(self):
        """
        This does a "timeless" version of RRT where the entire human 
        trajectory is a collision object at all times 
        """
        # do RRT
        graph_posns = np.array([self.curr_posn.copy()])
        parent_arr = [-1]
        n_pts_sampled = 0
        while True:
            n_pts_sampled += 1
            if n_pts_sampled > 120:
                return
            sample_pt = self.goal
            # sample a random point within RRT_MAX_LEN
            if np.random.random() < 0.8:
                # pick random point on [-2*pi, 2*pi]
                sample_pt = rand_posn()
            
            # find nearest point already in graph
            dists = np.linalg.norm(graph_posns - sample_pt, axis=1)
            close_idx = np.argmin(dists)
            close_pt, close_dist = graph_posns[close_idx,:], dists[close_idx]
            if close_dist > RRT_MAX_LEN:
                sample_pt = close_pt + (RRT_MAX_LEN/close_dist)*\
                    (sample_pt - close_pt)
                    
            # check for collisions
            if is_segment_valid((close_pt, sample_pt), self.curr_pred):
                # add to graph:
                sample_pt = sample_pt.reshape((1, -1))
                graph_posns = np.concatenate((graph_posns, sample_pt), axis=0)
                parent_arr.append(close_idx)
                if np.all(sample_pt == self.goal):
                    # solve graph by backtracking
                    soln = [sample_pt.ravel()]
                    next_idx = close_idx
                    while next_idx != -1:
                        soln.append(graph_posns[next_idx,:])
                        next_idx = parent_arr[next_idx]
                    soln = soln[::-1] # reverse so we go start -> goal
                    soln, times_arr = shortcut_soln(soln, self.curr_pred)
                    soln = soln[1:] # remove current position from soln
                    self.soln = np.array(soln)
                    
                    # construct JointTrajectory to send to controller
                    traj = JointTrajectory()
                    for idx, pt in enumerate(soln):
                        jt_pt = JointTrajectoryPoint()
                        jt_pt.positions = list(pt)
                        jt_pt.time_from_start = rospy.Duration(times_arr[idx])
                        traj.points.append(jt_pt)
                    self.ctrl_pub.publish(traj)
                    return

if __name__ == "__main__":
    rospy.init_node("rrt_baseline")
    goal = positions.posns[int(sys.argv[1])]
    # goal = np.array([1.0656882781191814, 0.4604144797878419, -3.118373350993719, -1.2922323399336983, -0.1051192292831713, -1.3377377734798825, 1.806642277664842])
    baseline = RRTBaseline(goal)
    rospy.spin()