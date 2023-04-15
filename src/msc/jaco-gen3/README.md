# CoMOTO Scripts
This is a collection of testing and other scripts for CoMOTO.

## Camera
The `camera` directory contains scripts for the camera calibration/setup. 
- `calibrate_camera.py` calibrates the transformation between the camera and jaco frame by having the user hold onto the jaco bracelet link and manually move the arm to a number of positions. A least-squares tranformation matrix is then solved between the fk and camera positions.
- `position_camera_arm.py` isn't going to be used, but it moves the arm to a pre-specified joint position so the camera can be held in a consistent place.

## controller
This directory contains test scripts for the adaptive controller, and
the controller itself.

## gen3.urdf
This is a model of the arm with no visual or collision components (for fk and ik)

## julia_tests
These are testing scripts for julia comoto.
- `broadcast_point_skel.py` sends trajectory predictions where all human joints are predicted to stay at a single point for the entire trajectory - useful for sanity testing
- `collapse_rt_traj.py` brings up a node that listens on the adaptive control topic and assembles the points into a single trajectory as the adaptive controller would have done, then prints and dispatches that trajectory when an empty message is posted to `/right_arm/traj_collapse/go`
- `send_gazebo_traj.py` can be used to visualize a trajectory from `collapse_rt_traj.py` without having to re-solve it every time

# Example workflows
## Sanity test

