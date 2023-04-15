# prediction\_pipeline
This is a ROS package containing the stages of the pipeline that precede the Julia CoMOTO implementation. It could, however, be used for any pose prediction project. The package contains:

- Instructions (NOT an automated script) for installing a fast pose estimation package (tested with the Asus Xtion Pro Live depth camera) (`install_procedure.sh`)
- ROS nodes for using pose prediction on the estimated pose trajectories
- Launch files to bring up the entire pose prediction portion of a pipeline in a single command

## Jaco Gen3 controller
This package also has a joint-space trajectory controller for the Jaco Gen3 robot. If you don't want to build that, you can edit the package.xml and CMakeLists.txt to remove all dependencies except rospy :)