"""
Adjust a human trajectory by a certain bias
"""
import numpy as np
import csv

bias_vec = np.array([0.1, -0.1, 0])
joints = ["right_elbow", "right_wrist", "right_palm"]

def apply_bias(traj, bias, changed_joints):
    """
    traj: human trajectory - numpy array with shape (n_timesteps, n_joints, 3)
    bias: changes to apply to trajectory - numpy array
        with shape (n_changed_joints, 3) or (3) (to apply same change to 
        all changed joints)
    changed_joints: indices of joints to change - numpy array of 
        shape (n_changed_joints) and dtype int

    returns a version of traj in which the positions of the joints in
        changed_joints are translated by bias in the final timestep
        degree of translation increases quadratically between timesteps
    """
    traj = np.copy(traj)
    n_timesteps = traj.shape[0]
    
    # make error increase quadratically from idx 1
    n_inc = int(n_timesteps*(n_timesteps-1)/2)
    inc = bias/n_inc

    scale = 0
    for i in range(n_timesteps):
        traj[i,changed_joints] += scale*inc
        scale += i + 1

    return traj

def uniq_jointnames(jnames):
    """
    Joint names is a list of csv headers which contains joint coordinate names
        (eg: headx, heady). Returns a list of unique joint names (["head"])
    """
    ret_val = []
    seen = set()
    for x in jnames:
        x = x[:-1]
        if x not in seen:
            ret_val.append(x)
            seen.add(x)
    return ret_val

def read_traj_file(fname):
    """
    Reads in file fname and returns a human trajectory and joint names

    Return: numpy array of shape (n_timesteps, n_joints, 3), array of strings
    """
    ret_val = []
    
    with open(fname, 'r') as f:
        reader = csv.reader(f)
        header = reader.next() #
        for row in reader:
            frow = [float(x) for x in row]
            pointarr = [[frow[i], frow[i+1], frow[i+2]] for i in range(0, len(frow), 3)]
            ret_val.append(pointarr)

    return np.array(ret_val), header

def write_traj(traj, new_fname, header=None):
    """
    Writes trajectory to a file new_fname

    traj: numpy array of shape (n_timesteps, n_joints, 3)
    """
    lines = []
    if header is not None:
        lines.append(",".join(header)+"\n")
    for i in range(traj.shape[0]):
        str_arr = [str(x) for x in traj[i].ravel()]
        lines.append(",".join(str_arr)+"\n")
    print(lines)
    with open(new_fname, 'w') as f:
        f.writelines(lines)

def main():
    traj, header = read_traj_file("means.csv")
    joint_names = uniq_jointnames(header)
    joint_idxs = [joint_names.index(j) for j in joints]
    new_traj = apply_bias(traj, bias_vec, joint_idxs)
    write_traj(new_traj, "means2.csv", header)
    
if __name__ == "__main__":
    main()