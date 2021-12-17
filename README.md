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

# Install yamaopt. The latest document is https://github.com/HiroIshida/yamaopt
# apt install for Python 2.x scikit-robot
sudo apt-get install -y libspatialindex-dev freeglut3-dev libsuitesparse-dev libblas-dev liblapack-dev
# If you don't have GLIBCXX_3.4.26
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install gcc-9
sudo apt install libstdc++6
cd yamaopt
pip install -e .
cd ..

# Install yamaopt_ros
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

    TODO

    ```
    xxx
    ```

  - Debug

    accumulating polygons

    ```
    roslaunch yamaopt_ros debug_accum_polygons.launch robot:=pr2
    roslaunch yamaopt_ros debug_accum_polygons.launch robot:=fetch
    ```

    visualizing polygons

    ```
    rosrun yamaopt_ros debug_visualize_polygons.py
    ```
