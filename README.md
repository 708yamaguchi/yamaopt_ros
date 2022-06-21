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

## Sample

1. Download sample rosbag

```
rosrun yamaopt_ros download_sample_data.py
```

2. launch sample

  - Sample

    Sensor placement optimization with real PR2 robot based on vision planes robot-fixed planes.

    ```
    roslaunch yamaopt_ros pr2_sensor_placement.launch use_base:=false
    ```

    Sensor placement optimization with Kinematics Simulator.

    ```
    # 1. Set use_vision_plances to false because we cannot use vision data with Kinematics Simulator
    # 2. Because pub-robot-plane-pr2.l and utils.l cannot communicate with each other about *ri* information in Kinematics Simulator mode,
    #    we need to give fixed robot pose to pub-robot-plane-pr2.l according *ri* pose in utils.l code.
    # 3. Give sensor_type because /sensor_type topic is not published in this case.
    roslaunch yamaopt_ros pr2_sensor_placement.launch use_base:=false use_vision_planes:=false fix_robot_planes:=true sensor_type:=ai_camera
    ```

    Sensor placement optimization via rosservice using PR2 rosbag (old?)

    ```
    roslaunch yamaopt_ros sample_pr2_sensor_placement.launch
    ```

    Sensor placement optimization via rosservice with fixed polygons (old?)

    ```
    roslaunch yamaopt_ros sample_fixed_polygon_sensor_placement.launch
    ```
