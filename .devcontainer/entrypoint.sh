set -e

CATKIN_DIR="\$HOME/fav/catkin_ws"
PX4_DIR="\$HOME/fav/fav_PX4-Autopilot"

echo "source $CATKIN_DIR/devel/setup.bash" >> ~/.bashrc

sudo apt-get update

rosdep update

rosdep install --from-path src --ignore-src -y -r

echo "source $CATKIN_DIR/devel/setup.bash" >> ~/.bashrc

echo "source $PX4_DIR/Tools/setup_gazebo.bash $PX4_DIR $PX4_DIR/build/px4_sitl_default > /dev/null" >> ~/.bashrc

echo "export ROS_IP=127.0.0.1" >> ~/.bashrc

echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$PX4_DIR" >> ~/.bashrc

echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$PX4_DIR/Tools/sitl_gazebo" >> ~/.bashrc


