# yamaopt

ROS wrapper for [yamaopt](https://github.com/HiroIshida/yamaopt), optimizing the position where the robot attaches the sensor.

## Setup

Create ROS workspace for yamaopt.
```
# Download yamaopt and yamaopt_ros
mkdir yamaopt_ws/src -p
cd yamaopt_ws/src
wstool init .
wstool merge -t . https://raw.githubusercontent.com/708yamaguchi/yamaopt_ros/master/yamaopt.rosinstall
wstool update -t .

# Prepare for Installing yamaopt. The latest document is https://github.com/HiroIshida/yamaopt
# apt install for Python 2.x scikit-robot
sudo apt-get install -y libspatialindex-dev freeglut3-dev libsuitesparse-dev libblas-dev liblapack-dev
# If you don't have GLIBCXX_3.4.26
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install gcc-9
sudo apt install libstdc++6

# Install yamaopt and yamaopt_ros
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install -y -r --from-paths . --ignore-src
cd ../
catkin build yamaopt_ros
source devel/setup.bash
```

## Usage

### PR2 Demo

  soup\_from\_boil demo.

  - Before launching the following file, `(setup)` function in `soup-from-boil.l` must be executed.
  - If rosbag arg is not given, rosbag is not recorded.
  - Replace the right finger of PR2 with a finger for grasping the ladle.
  - Place vinyl on the base of PR2 (for waterproofing)
  - Do not draw thermography data on the m5stack screen to save battery power.
  - Make sure that PR2's c2 has bluetooth module and has rfcomm devices for thermography.
    ```
    ssh pr1040s
    rfcomm bind 0 AA:BB:CC:DD:EE:FF
    rfcomm # list up rfcomm devices
    ```

  Main program
  ```
  roslaunch yamaopt_ros soup_from_boil.launch
  ```

  Rosbag record. Please run this inside pr2.
  ```
  ssh pr1040s
  roslaunch yamaopt_ros pr2_rosbag_record.launch rosbag:=/removable/bagfiles/$(date +%Y-%m%d-%H%M%S).bag
  ```

### PR2 Sample

  Sensor placement optimization with real PR2 robot based on vision planes robot-fixed planes.

  ```
  roslaunch yamaopt_ros pr2_sensor_placement.launch use_base:=false
  ```

  Sensor placement optimization with Kinematics Simulator.

  ```
  # 1. Set use_vision_plances to false because we cannot use vision data with Kinematics Simulator
  # 2. Because pub-robot-plane-pr2.l and utils.l cannot communicate with each other about *ri* information in Kinematics Simulator mode,
  #    we need to give fixed robot pose to pub-robot-plane-pr2.l according *ri* pose in pr2-place-sensor.l code.
  # 3. Give sensor_type because /sensor_type topic is not published in this case.
  roslaunch yamaopt_ros pr2_sensor_placement.launch use_base:=false use_vision_planes:=false fix_robot_planes:=true sensor_type:=ai_camera
  ```

  Record rosbag

  ```
  roslaunch yamaopt_ros pr2_rosbag_record.launch rosbag:=$HOME/$(date +%Y-%m%d-%H%M%S).bag
  ```

  Play rosbag with optimization visualization

  ```
  # We need to give sensor_type if /sensor_type topic is not recorded
  # For other args (e.g. use_base, arm, ...), we need to specify the same args
  # as we used when recording rosbag.
  roslaunch yamaopt_ros pr2_rosbag_play.launch sensor_type:=ai_camera use_base:=false rosbag:=$HOME/test.bag
  ```

  Sensor placement optimization via rosservice using PR2 rosbag (old?)

  ```
  roslaunch yamaopt_ros sample_pr2_sensor_placement.launch
  ```

  Sensor placement optimization via rosservice with fixed polygons (old?)

  ```
  roslaunch yamaopt_ros sample_fixed_polygon_sensor_placement.launch
  ```

### Fetch Sample

  Sensor placement optimization with Kinematics Simulator.

  ```
  # 1. Set use_vision_plances to false because we cannot use vision data with Kinematics Simulator
  # 2. Because pub-robot-plane-fetch.l and utils.l cannot communicate with each other about *ri* information in Kinematics Simulator mode,
  #    we need to give fixed robot pose to pub-robot-plane-fetch.l according *ri* pose in fetch-place-sensor.l code.
  # 3. Give sensor_type because /sensor_type topic is not published in this case.
  roslaunch yamaopt_ros fetch_sensor_placement.launch use_base:=false use_vision_planes:=false fix_robot_planes:=true sensor_type:=ai_camera
  ```

  Record rosbag

  ```
  roslaunch yamaopt_ros fetch_rosbag_record.launch rosbag:=$HOME/$(date +%Y-%m%d-%H%M%S).bag
  ```

  Play rosbag with optimization visualization

  ```
  # We need to give sensor_type if /sensor_type topic is not recorded
  # For other args (e.g. use_base, arm, ...), we need to specify the same args
  # as we used when recording rosbag.
  roslaunch yamaopt_ros fetch_rosbag_play.launch sensor_type:=ai_camera use_base:=false rosbag:=$HOME/test.bag
  ```
