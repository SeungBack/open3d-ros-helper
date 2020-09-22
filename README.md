# open3d-ros-helper
- helper for jointly using open3d and numpy in ROS
- currently, supports only XYZ point cloud

## Dependencies
- python 2.7
- open3d == 0.9


## Features
#### convert_ros_to_o3d
- convert ros pointcloud to open3d cloud
- `sensor_msg.msg.PointCloud2` ⮕ `open3d.geometry.PointCloud`
#### convert_o3d_to_ros
- convert open3d pointcloud to ros cloud
- `open3d.geometry.PointCloud` ⮕ `sensor_msg.msg.PointCloud2`
#### do_transform_o3d_cloud
- transform a input cloud with respect to the specific frame 
- open3d version of tf2_geometry_msgs.do_transform_point
- `open3d.geometry.PointCloud`, `geometry_msgs.msgs.TransformStamped' ⮕ `open3d.geometry.PointCloud`
#### o3d_cloud_pass_through_filter
- apply a pass through filter with ROI 
- reference: https://github.com/powersimmani/example_3d_pass_through-filter_guide
- `open3d.geometry.PointCloud`, {'x': [min, max], 'y': [min, max], 'z': : [min, max]} ⮕ `open3d.geometry.PointCloud`
#### crop_o3d_cloud_with_mask
- crop input cloud according to the mask and camera info
- `open3d.geometry.PointCloud`, `numpy.ndarray (uint16)`, `sensor_msgs.msgs.CameraInfo` ⮕ `open3d.geometry.PointCloud`
#### icp_refinement
- do point-to-point ICP registration implemented in open3d
- `open3d.geometry.PointCloud`, `open3d.geometry.PointCloud` ⮕ `open3d.registration.RegistrationResult`
#### icp_refinement_with_ppf_match
- do ICP registration using ppf matching implemented in opencv
- `open3d.geometry.PointCloud`, `open3d.geometry.PointCloud` ⮕ `numpy.ndarray (4x4)`


## Future Work
- support XYZRGB point cloud

## References
- pcl_herlper: https://github.com/udacity/RoboND-Perception-Exercises




