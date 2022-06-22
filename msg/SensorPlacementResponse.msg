# Copied from SensorPlacement.srv
std_msgs/String[] joint_names # Names of robot joints to move
std_msgs/Float32[] angle_vector # Robot angle vector corresponding to joint_names
geometry_msgs/Pose base_pose # Robot base pose after movement
std_msgs/String axis # Module optimization axis (e.g. x, y, z)
std_msgs/Bool success # Whether sqp optimization is succeeded or not
