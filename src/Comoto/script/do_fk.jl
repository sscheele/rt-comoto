import Pkg; Pkg.activate("..");
using StaticArrays;
using Comoto: get_probinfo, ComotoParameters, RobotInfo;

const OBJECT_SET = SArray{Tuple{3,2}}(reshape([0.752,-0.19,0.089, 0.752, 0.09, -0.089], (3,2)));
const OBJECT_POS = OBJECT_SET[:,1];

function get_fk_fn()
    joint_start = @SVector [-0.7240388673767146, -0.34790398102066433, 2.8303899987665897, -2.54032606205873, 1.3329587647643253, 2.7596249683074614, 0.850582268802067]
    # joint_target = [0.04506347261090404, 1.029660363493563, -0.0563325987175789, -1.8024937659056217, 0.14645022654203643, 0.3406148976556631, -0.12291455548612884] #near reaching case
    joint_target = @SVector [-0.3902233335085379, 1.7501020413442578, 0.8403277122861033, -0.22924505085794067, 2.8506926562622024, -1.417026666483551, -0.35668663982214976] #far reaching case
    n_timesteps = 20;
    dt = 0.25;

    params = ComotoParameters(
        joint_start,
        joint_target,
        n_timesteps,
        dt,
        OBJECT_SET
    )

    jaco_joints = ["j2s7s300_link_1", "j2s7s300_link_2", "j2s7s300_link_3", "j2s7s300_link_4", 
        "j2s7s300_link_5", "j2s7s300_link_6", "j2s7s300_link_7"];
    jaco_info = RobotInfo(
        jaco_joints,
        "novis-jaco.urdf",
        "j2s7s300_ee_base",
        7
    )
    base_prob_info = get_probinfo(params, jaco_info, "human_trajs/669-means-fmt.csv", 
        "human_trajs/669-vars-fmt.csv");
    return base_prob_info.eef_fk;
end

fk = get_fk_fn();
posn = [2.2410335457029205, 2.816951186180262, 3.5541107138047776, 1.3378698662699502, 6.076088313435101, 4.022068863696378, 5.031459114793775];

println(fk(posn));