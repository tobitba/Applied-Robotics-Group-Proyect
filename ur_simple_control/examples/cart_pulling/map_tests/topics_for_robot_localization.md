robot localization package
==========================
tf tree requirements
--------------------
There are 3 frames of interest
1. base_link - rigidly fixed to the robot
2. odom - world frame, typically aligned with robot's start position
3. map - world frame

The robot localization's output is a pose given in either the odom or the map frame,
and a velocity given in the base_link frame.

Topics that send the following messages are of interest:
1. nav_msgs/Odometry
2. geometry_msgs/PoseWithCovarianceStamped
3. geometry_msgs/TwistWithCovarianceStamped
4. sensor_msgs/Imu - frame in which it is given is not standardized - robot localization expects ENU frame.
    However, hopefully the IMU message has a frame_id field specifying the frame. 
    In this case robot localization maps it to the map or the odom frame.


heron
------
1. /amcl_pose
    Type: geometry_msgs/msg/PoseWithCovarianceStamped
    0.668 Hz, but the robot needs to move for this to be published
2. /base_pose_ground_truth
    Type: nav_msgs/msg/Odometry
    50 Hz
3. /imu_data/data
    Type: sensor_msgs/msg/Imu
        -> should have frame_id in the header 
    50 Hz
4. /initialpose
    Type: geometry_msgs/msg/PoseWithCovarianceStamped
    pointless
5. /odom
    Type: nav_msgs/msg/Odometry
    1000 Hz

mobile yumi
-----------
1. /maskinn/navigation/amcl_pose
    Type: geometry_msgs/msg/PoseWithCovarianceStamped
    5.712 Hz, but the robot needs to move for this to be published
2. /maskinn/navigation/initialpose
    Type: geometry_msgs/msg/PoseWithCovarianceStamped
    pointless
3. /maskinn/platform/odometry
    Type: nav_msgs/msg/Odometry
    10 Hz
4. /maskinn/sensors/camera/imu
    Type: sensor_msgs/msg/Imu
        -> should have frame_id in the header 
    91.195 Hz
