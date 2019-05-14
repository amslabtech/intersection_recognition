# intersection_recognition

# Requirements
- Eigen3
- PCL(>=1.7)
- amsl_navigation_managers
- amsl_navigation_msgs

# Environment
- Ubuntu16.04 & ROS kinetic
- Ubuntu18.04 & ROS melodic

# Nodes
## calc_shape_savecloud
### Published topics
- /detect_shape2 (sensor_msgs/PointCloud2)
### Subscribed topics
- /save_cloud (sensor_msgs/PointCloud2)
- /lcl (nav_msgs/Odometry)

## detect_peak_save_pc
### Published topics
- /detect_peak2 (sensor_msgs/PointCloud2)
- /visualization_marker/long (visualization_msgs/Marker)
- /visualization_marker/short (visualization_msgs/Marker)
- /peak/deg (std_msgs::Int32MultiArray)
- /road/deg (std_msgs::Int32MultiArray)
### Subscribed topics
- /detect_shape2 (sensor_msgs/PointCloud2)

## intersection_matching
### Published topics
- /intersection_flag (std_msgs/Bool)
- /target_yaw (std_msgs/Float64)
- /next_branch (visualization_msgs/MarkerArray)
- /mode (std_msgs::Int16)
### Subscribed topics
- /peak/deg (std_msgs/Int32MultiArray)
- /local_node (std_msgs/Int16MultiArray)
- /lcl (nav_msgs/Odometry)
- /odom_gps (nav_msgs/Odometry)
- /start (std_msgs::Bool)
- /turn (std_msgs::Bool)

## normal_curvature_pc_bottom
### Published topics
- /static_clout (sensor_msgs/PointCloud2)
- /curvature_cloud (sensor_msgs/PointCloud2)
### Subscribed topics
- /perfect_velodyne/normal (sensor_msgs/PointCloud2)

## save_velodyne_normal_pc_odom
### Published topics
- /save_cloud (sensor_msgs/PointCloud2)
- /zed_debug (sensor_msgs/PointCloud2)
### Subscribed topics
- /local_cloud (sensor_msgs/PointCloud2)
- /lcl (nav_msgs/Odometry)
- /zed_grasspoints (sensor_msgs/PointCloud2)
- /velodyne_points/inte (sensor_msgs/PointCloud2)
- /flag/diff (std_msgs/Float64)
