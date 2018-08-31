# ERA: EPOCHS Reference Application


## Requirements

ERA requires:
- Ubuntu 16.04
- Python 2.7
- GNU gcc/g++ 5.4
- ROS Kinetic and Gazebo 7
- cmake 3.5.0+
- Cartographer (automatically installed using the `install_isolated.sh` script explained below)

Required ROS packages:
- kinetic-desktop-full
- kinetic-openslam-gmapping
- ~~kinetic-gmapping~~ (Included in src)
- kinetic-turtlebot
- kinetic-kobuki
- kinetic-robot-state-publisher
- kinetic-robot-pose-publisher
- kinetic-joint-state-publisher

Some subset of these additional ROS packages is required to teleoperate the robots and perform sensing:
 - kinetic-kobuki-controller-tutorial
 - kinetic-kobuki-core
 - kinetic-kobuki-description
 - kinetic-kobuki-desktop
 - kinetic-kobuki-gazebo
 - kinetic-kobuki-gazebo-plugins
 - kinetic-kobuki-keyop
 - kinetic-kobuki-node

Required for running on a VM:
- No ssh: Use 'Normal Start' and a desktop environment
- Gazebo 7.4+
- libignition-math2-dev
- LIBGL_ALWAYS_SOFTWARE=1


## Installation

If not done yet, source the ROS environment setup file:

```
source /opt/ros/kinetic/setup.bash
```

> In addition to the ERA packages, the `install_isolated.sh` script used in the sections below will automatically install <a href="https://google-cartographer-ros.readthedocs.io/en/latest/#" target="_blank">Cartographer</a>, a library for real-time 2D and 3D simultaneous localization and mapping (SLAM).

### Fresh Install with No Preexisting Catkin Workspace

```
cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone https://github.ibm.com/ERA/era_gazebo.git
cd ~/catkin_ws
./install_isolated.sh
```

See <a href="https://help.github.com/enterprise/2.10/user/articles/which-remote-url-should-i-use/#cloning-with-https-urls-recommended" target="_blank">this page</a> if you find issues with the `git clone` command.


### Installation into an Existing Workspace

```
cd catkin_ws/src
git clone https://github.ibm.com/ERA/era_gazebo.git
cd ~/catkin_ws
./install_isolated.sh
```

Launch the workload:

```
roslaunch era_gazebo era.launch
```
Note: RViz will not run in a headless environment, so if you want to run in a visual environment, ensure that you launch the ROS workload from a desktop environment.

To launch the workload **without Gazebo's GUI**:

```
roslaunch era_gazebo era.launch gui:=false
```


## Moving and Controlling the TurtleBots

Once running, TurtleBots can be teleoperated using the keyboard. Each bot is controlled independently; therefore, for each TurtleBot, open a separate terminal and execute the following command:

```
roslaunch era_gazebo keyboard_teleop.launch namespace:=<robot_id>
```

Replace `<robot_id>` with the ID of the robot to be controlled, like `r0` and `r1`.

## Automatically Profiling the ERA

To generate a sample workload using rosbag:

```
roslaunch era_gazebo era_auto.launch workload_path:=/path/to/bags workload_size:=[small|large] workload_type:=[rotator|wanderer] profiles_path:=/path/to/profiles project_path:=<default=.>
```

The duration of the recorded trace (bag) is determined by specifying `small` or `large` for the `workload_size` parameter. The behavior of the workload is determined by specifying `rotator` or `wanderer` for the `workload_type`. The `profiles_path` during this stage just sets up the correct directories for profiling. The `project_path` should point to the base directory of the ERA application.

To playback and profile a generated workload using rosbag and callgrind:

```
roslaunch era_gazebo era_auto_callgrind.launch workload_path:=/path/to/bags workload_size:=[small|large] workload_type:=[rotator|wanderer] profiles_path:=/path/to/profiles project_path:=<default=.>
```

The workload trace (bag) used to profile is determined by the `workload_size` and `workload_type` parameters. The output of the profiling (one per profiled node) is output to the `profiles_path`.


## Gmapping

ERA makes use of <a href="http://wiki.ros.org/gmapping" target="_blank">gmapping</a> to conduct laser-based SLAM (Simultaneous Localization and Mapping) and create 2-D occupancy grid maps. A modified version of gmapping is included with ERA. This gmapping version generates 2-D occupancy grid maps with probability values (0-100%) instead of the default *unknown*, *free* and *occupied* grid values.


## Adjusting the Camera Frame Rate

In simulation mode (i.e. when using Gazebo to simulate the world and robots), the frame rate of the Kinect depth camera attached to Turtlebots can be adjusted by editing the `update_rate` element in the corresponding Xacro file (e.g. `turtlebot_gazebo.urdf.xacro`):

```
<xacro:macro name="turtlebot_sim_3dsensor">
  <gazebo reference="camera_link">  
    <sensor type="depth" name="camera">
      <update_rate>30.0</update_rate>
      ...
```

Also, be sure to set the plugin's update rate to 0 in order to allow the parent `<sensor>` tag to control the frame rate:

```
<plugin name="kinect_camera_controller" filename="libgazebo_ros_openni_kinect.so">
  <updateRate>0</updateRate>
  ...
```
