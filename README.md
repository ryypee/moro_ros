# moro_ros

The `moro_ros` metapackage contains all the necessary ROS packages for the Tampere University course IHA-4306 Fundamentals of Mobile Robots.

## Dependencies

1. Install ROS Kinetic by following the [instructions][kinetic]. The ROS-Base installation is sufficient.
2. Install [Stage][stage]
   ```bash
   sudo apt-get install -y git cmake g++ libfltk1.3-dev libjpeg8-dev libpng12-dev \
      libglu1-mesa-dev libltdl-dev
   git clone https://github.com/tvalimaki/Stage.git ~/Stage
   cd ~/Stage && git checkout patch-fiducial
   mkdir build && cd build
   cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO ..
   make && sudo make install
   ```
3. Create a Catkin workspace
   ```bash
   sudo apt-get install -y python-catkin-tools
   mkdir -p ~/catkin_ws/src && cd ~/catkin_ws
   catkin init
   ```
4. Clone [`stage_ros`][stage_ros] to `src` in your Catkin workspace
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/tuw-robotics/stage_ros.git
   ```
5. Clone [`marker_rviz_plugin`][rviz_plugin] to `src` in your Catkin workspace, and switch to branch `patch-1`
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/tvalimaki/marker_rviz_plugin.git
   cd marker_rviz_plugin && git checkout patch-1
   ```
6. Install `pip`
   ```bash
   sudo apt-get install -y python-pip
   ```

## Installation

1. Clone the repository to `src` in your Catkin workspace
2. Install dependencies using `rosdep`
   ```bash
   rosdep install -y --from-paths src --ignore-src --skip-keys=stage
   ```
3. Build using Catkin

[kinetic]: http://wiki.ros.org/kinetic/Installation/Ubuntu
[stage]: https://github.com/tvalimaki/Stage
[stage_ros]: https://github.com/tuw-robotics/stage_ros
[rviz_plugin]: https://github.com/tvalimaki/marker_rviz_plugin
