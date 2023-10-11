# rt-comoto
This is a ROS package containing the rt-comoto pipeline. It consists of roughly 5 components:

- Human pose tracking, which is responsible for identifying the single person who will be tracked and reporting the positions of their key points in space, as well as for camera calibration
- Human pose prediction, which can be done with neural network approaches such as PGBIG or classical approaches like curve-fitting
- RT-CoMOTO problem creation, which accumulates information about the environment, formats it as an RT-CoMOTO problem, and passes it to the solution service
- RT-CoMOTO problem solution, written in Julia using ALTRO
- Control of the Jaco Gen3 arm, implemented such that MPC windows can interrupt each other without creating hesitation or jerky motion

The repository additionally provides reference implementations of RRT, legible trajectories (per Dragan et al), speed-adaptive control, and nominal control, to be used as baselines for a user study.

Finally, we provide scripts such as user_study.py, which are designed to give an easy entrypoint to test our algorithm against the baselines. user_study.py will conduct one round of a user study, recording both video and joint positions.

## Jaco Gen3 controller
This package also has a C/C++ joint-space trajectory controller for the Jaco Gen3 robot. It isn't necessary, as we ended up changing the control technique and never translated the Python prototype to a more appropriate language, but it's here in case the Python controller stops working. If you don't want to build it, you can edit the package.xml and CMakeLists.txt to remove all dependencies except rospy :)
