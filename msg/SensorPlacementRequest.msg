# Copied from SensorPlacement.srv
std_msgs/String[] joint_names # Names of robot joints to move
std_msgs/Float32[] angle_vector # Current robot angle vector corresponding to joint_names
geometry_msgs/Point target_point # Target point to be observed by the sensor
jsk_recognition_msgs/PolygonArray polygon_array # Input polygon array
jsk_recognition_msgs/ModelCoefficientsArray coefficients # Input polygon coefficients
geometry_msgs/PolygonStamped movable_polygon # Movable area of robot. Currently single polygon is allowed
