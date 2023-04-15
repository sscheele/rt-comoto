sudo apt install libudev-dev ros-melodic-openni2-camera ros-melodic-openni-camera
mkdir ~/tracker
cd ~/tracker
git clone https://github.com/structureio/OpenNI2.git
cd OpenNI2
make -j`nproc`

wget http://jaist.dl.sourceforge.net/project/roboticslab/External/nite/NiTE-Linux-x64-2.2.tar.bz2
extract NiTE-Linux-x64-2.2.tar.bz2
cd NiTE-Linux-x64-2.2
sudo ./install.sh

cd ~/catkin_ws/src
git clone https://github.com/Reilif/skeleton_tracker

# edit CMakeLists.txt for skeleton_tracker:
# set(OPENNI2_DIR ~/tracker/OpenNI2)
# set(OPENNI2_WRAPPER /opt/ros/melodic/include/openni2_camera/)
# set(NITE2_DIR ~/tracker/NiTE-Linux-x64-2.2/)
# set(NITE2_LIB ~/tracker/NiTE-Linux-x64-2.2/Redist/libNiTE2.so)

cd ~/catkin_ws
catkin_make skeleton_tracker_generate_messages
catkin_make

