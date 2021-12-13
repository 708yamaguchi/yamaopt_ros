# yamaopt

ROS wrapper for [yamaopt](https://github.com/HiroIshida/yamaopt), optimizing the position where the robot attaches the sensor.

# Setup

Create ROS workspace for yamaopt.
```
# Download yamaopt and yamaopt_ros
mkdir yamaopt_ws/src -p
cd yamaopt_ws/src
wstool init .
wstool merge -t . https://raw.githubusercontent.com/708yamaguchi/yamaopt_ros/master/yamaopt.rosinstall
wstool update -t .

# Install yamaopt
# apt install for Python 2.x scikit-robot
sudo apt-get install -y libspatialindex-dev freeglut3-dev libsuitesparse-dev libblas-dev liblapack-dev
cd yamaopt
pip install -e .

# Install yamaopt_ros
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install -y -r --from-paths . --ignore-src
cd ../
catkin build yamaopt_ros
source devel/setup.bash
```

