# open3d-ros-helper

[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/Naereen/StrapDown.js/graphs/commit-activity)
[![PyPI version](https://badge.fury.io/py/open3d-ros-helper.svg)](https://badge.fury.io/py/open3d-ros-helper)
[![PyPI license](https://img.shields.io/pypi/l/ansicolortags.svg)](https://pypi.python.org/pypi/open3d-ros-helper/)
[![Hits](https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2FSeungBack%2Fopen3d-ros-helper&count_bg=%2379C83D&title_bg=%23555555&icon=&icon_color=%23E7E7E7&title=hits&edge_flat=false)](https://hits.seeyoufarm.com)
[![Python 2.7](https://img.shields.io/badge/python-2.7-blue.svg)](https://www.python.org/downloads/release/python-270/)
[![contributions welcome](https://img.shields.io/badge/contributions-welcome-brightgreen.svg?style=flat)](https://github.com/SeungBack/open3d-ros-helper/issues)


- Helper for jointly using open3d and ROS
- Easy conversion between ROS and open3d point cloud (supports both XYZ & XYZRGB point cloud)
- Easy conversion between ROS pose and transform 

## Dependencies
- python 2.7
- ros-numpy
- open3d == 0.9 

## Installation
```
$ sudo apt install ros-melodic-ros-numpy
$ pip2 install numpy open3d==0.9.0 opencv-python==4.2.0.32 pyrsistent==0.13
$ pip2 install open3d_ros_helper
```

## Usage

Import `open3d-ros-helper`
```
from open3d_ros_helper import open3d_ros_helper as orh
```

Convert `4x4 SE(3)` to `geometry_msgs/Transform`
```
import numpy as np
se3 = np.eye(4)
ros_transform = orh.se3_to_transform(se3) 
```

Convert `sensor.msg.PointCloud2` to `open3d.geometry.PointCloud`
```
o3dpc = orh.rospc_to_o3dpc(some_ros_pointcloud) 
```

Convert `open3d.geometry.PointCloud` to `sensor.msg.PointCloud2`
```
rospc = orh.rospc_to_o3dpc(o3dpc) 
```

## Authors
* **Seunghyeok Back** [seungback](https://github.com/SeungBack)

## License
This project is licensed under the MIT License


## References
Some codes are rewritten from
- [pcl_herlper](https://github.com/udacity/RoboND-Perception-Exercises)
- [conversion b/w ros transforms](https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/)
- [averaging-quaternion](https://github.com/christophhagen/averaging-quaternions)



# Documentation
## Table of contents
* [pose\_to\_pq](#open3d_ros_helper.pose_to_pq)
* [pose\_stamped\_to\_pq](#open3d_ros_helper.pose_stamped_to_pq)
* [transform\_to\_pq](#open3d_ros_helper.transform_to_pq)
* [transform\_stamped\_to\_pq](#open3d_ros_helper.transform_stamped_to_pq)
* [msg\_to\_se3](#open3d_ros_helper.msg_to_se3)
* [pq\_to\_transform](#open3d_ros_helper.pq_to_transform)
* [pq\_to\_transform\_stamped](#open3d_ros_helper.pq_to_transform_stamped)
* [se3\_to\_transform](#open3d_ros_helper.se3_to_transform)
* [se3\_to\_transform\_stamped](#open3d_ros_helper.se3_to_transform_stamped)
* [average\_q](#open3d_ros_helper.average_q)
* [average\_pq](#open3d_ros_helper.average_pq)
* [rospc\_to\_o3dpc](#open3d_ros_helper.rospc_to_o3dpc)
* [o3dpc\_to\_rospc](#open3d_ros_helper.o3dpc_to_rospc)
* [do\_transform\_point](#open3d_ros_helper.do_transform_point)
* [apply\_pass\_through\_filter](#open3d_ros_helper.apply_pass_through_filter)
* [crop\_with\_2dmask](#open3d_ros_helper.crop_with_2dmask)
* [p2p\_icp\_registration](#open3d_ros_helper.p2p_icp_registration)
* [ppf\_icp\_registration](#open3d_ros_helper.ppf_icp_registration)

<a name="open3d_ros_helper.pose_to_pq"></a>
#### pose\_to\_pq

```python
pose_to_pq(pose)
```

convert a ROS PoseS message into position/quaternion np arrays

**Arguments**:

- `pose` _geometry_msgs/Pose_ - ROS geometric message to be converted

**Returns**:

- `p` _np.array_ - position array of [x, y, z]
- `q` _np.array_ - quaternion array of [x, y, z, w]
  source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/

<a name="open3d_ros_helper.pose_stamped_to_pq"></a>
#### pose\_stamped\_to\_pq

```python
pose_stamped_to_pq(pose_stamped)
```

convert a ROS PoseStamped message into position/quaternion np arrays

**Arguments**:

- `pose_stamped` _geometry_msgs/PoseStamped_ - ROS geometric message to be converted

**Returns**:

- `p` _np.array_ - position array of [x, y, z]
- `q` _np.array_ - quaternion array of [x, y, z, w]
  source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/

<a name="open3d_ros_helper.transform_to_pq"></a>
#### transform\_to\_pq

```python
transform_to_pq(transform)
```

convert a ROS Transform message into position/quaternion np arrays

**Arguments**:

- `transform` _geometry_msgs/Transform_ - ROS geometric message to be converted

**Returns**:

- `p` _np.array_ - position array of [x, y, z]
- `q` _np.array_ - quaternion array of [x, y, z, w]
  source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/

<a name="open3d_ros_helper.transform_stamped_to_pq"></a>
#### transform\_stamped\_to\_pq

```python
transform_stamped_to_pq(transform_stamped)
```

convert a ROS TransformStamped message into position/quaternion np arrays

**Arguments**:

- `transform_stamped` _geometry_msgs/TransformStamped_ - ROS geometric message to be converted

**Returns**:

- `p` _np.array_ - position array of [x, y, z]
- `q` _np.array_ - quaternion array of [x, y, z, w]
  source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/

<a name="open3d_ros_helper.msg_to_se3"></a>
#### msg\_to\_se3

```python
msg_to_se3(msg)
```

convert geometric ROS messages to SE(3)

**Arguments**:

  msg (geometry_msgs/Pose, geometry_msgs/PoseStamped,
  geometry_msgs/Transform, geometry_msgs/TransformStamped): ROS geometric messages to be converted

**Returns**:

- `se3` _np.array_ - a 4x4 SE(3) matrix as a numpy array
  source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/

<a name="open3d_ros_helper.pq_to_transform"></a>
#### pq\_to\_transform

```python
pq_to_transform(p, q)
```

convert position, quaternion to geometry_msgs/Transform

**Arguments**:

- `p` _np.array_ - position array of [x, y, z]
- `q` _np.array_ - quaternion array of [x, y, z, w]

**Returns**:

- `transform` _geometry_msgs/Transform_ - ROS transform of given p and q

<a name="open3d_ros_helper.pq_to_transform_stamped"></a>
#### pq\_to\_transform\_stamped

```python
pq_to_transform_stamped(p, q, source_frame, target_frame, stamp=None)
```

convert position, quaternion to geometry_msgs/TransformStamped

**Arguments**:

- `p` _np.array_ - position array of [x, y, z]
- `q` _np.array_ - quaternion array of [x, y, z, w]
- `source_frame` _string_ - name of tf source frame
- `target_frame` _string_ - name of tf target frame

**Returns**:

- `transform_stamped` _geometry_msgs/TransformStamped_ - ROS transform_stamped of given p and q

<a name="open3d_ros_helper.se3_to_transform"></a>
#### se3\_to\_transform

```python
se3_to_transform(transform_nparray)
```

convert 4x4 SE(3) to geometry_msgs/Transform

**Arguments**:

- `transform_nparray` _np.array_ - 4x4 SE(3)

**Returns**:

- `transform` _geometry_msgs/Transform_ - ROS transform of given SE(3)

<a name="open3d_ros_helper.se3_to_transform_stamped"></a>
#### se3\_to\_transform\_stamped

```python
se3_to_transform_stamped(transform_nparray, source_frame, target_frame, stamp=None)
```

convert 4x4 SE(3) to geometry_msgs/TransformStamped

**Arguments**:

- `transform_nparray` _np.array_ - 4x4 SE(3)
- `source_frame` _string_ - name of tf source frame
- `target_frame` _string_ - name of tf target frame

**Returns**:

- `transform_stamped` _geometry_msgs/TransformStamped_ - ROS transform_stamped of given SE(3)

<a name="open3d_ros_helper.average_q"></a>
#### average\_q

```python
average_q(qs)
```

calculate the average of quaternions

**Arguments**:

- `qs` _np.array_ - multiple quaternion array of shape Nx4

**Returns**:

- `q_average` _np.array_ - averaged quaternion array
  source codes from https://github.com/christophhagen/averaging-quaternions

<a name="open3d_ros_helper.average_pq"></a>
#### average\_pq

```python
average_pq(ps, qs)
```

average the multiple position and quaternion array

**Arguments**:

- `ps` _np.array_ - multiple position array of shape Nx3
- `qs` _np.array_ - multiple quaternion array of shape Nx4

**Returns**:

- `p_mean` _np.array_ - averaged position array
- `q_mean` _np.array_ - averaged quaternion array

<a name="open3d_ros_helper.rospc_to_o3dpc"></a>
#### rospc\_to\_o3dpc

```python
rospc_to_o3dpc(rospc, remove_nans=False)
```

covert ros point cloud to open3d point cloud

**Arguments**:

- `rospc` _sensor.msg.PointCloud2_ - ros point cloud message
- `remove_nans` _bool_ - if true, ignore the NaN points

**Returns**:

- `o3dpc` _open3d.geometry.PointCloud_ - open3d point cloud

<a name="open3d_ros_helper.o3dpc_to_rospc"></a>
#### o3dpc\_to\_rospc

```python
o3dpc_to_rospc(o3dpc, frame_id=None, stamp=None)
```

convert open3d point cloud to ros point cloud

**Arguments**:

- `o3dpc` _open3d.geometry.PointCloud_ - open3d point cloud
- `frame_id` _string_ - frame id of ros point cloud header
- `stamp` _rospy.Time_ - time stamp of ros point cloud header

**Returns**:

- `rospc` _sensor.msg.PointCloud2_ - ros point cloud message

<a name="open3d_ros_helper.do_transform_point"></a>
#### do\_transform\_point

```python
do_transform_point(o3dpc, transform_stamped)
```

transform a input cloud with respect to the specific frame
open3d version of tf2_geometry_msgs.do_transform_point

**Arguments**:

- `o3dpc` _open3d.geometry.PointCloud_ - open3d point cloud
- `transform_stamped` _geometry_msgs.msgs.TransformStamped_ - transform to be applied

**Returns**:

- `o3dpc` _open3d.geometry.PointCloud_ - transformed open3d point cloud

<a name="open3d_ros_helper.apply_pass_through_filter"></a>
#### apply\_pass\_through\_filter

```python
apply_pass_through_filter(o3dpc, x_range, y_range, z_range)
```

apply 3D pass through filter to the open3d point cloud

**Arguments**:

- `o3dpc` _open3d.geometry.PointCloud_ - open3d point cloud
- `x_range` _list_ - list of [x_min, x_maz]
- `y_range` _list_ - list of [y_min, y_maz]
- `z_range` _list_ - list of [z_min, z_max]

**Returns**:

- `o3dpc` _open3d.geometry.PointCloud_ - filtered open3d point cloud
  some codes from https://github.com/powersimmani/example_3d_pass_through-filter_guide

<a name="open3d_ros_helper.crop_with_2dmask"></a>
#### crop\_with\_2dmask

```python
crop_with_2dmask(o3dpc, mask)
```

crop open3d point cloud with given 2d binary mask

**Arguments**:

- `o3dpc` _open3d.geometry.PointCloud_ - open3d point cloud
- `mask` _np.array_ - binary mask aligned with the point cloud frame

**Returns**:

- `o3dpc` _open3d.geometry.PointCloud_ - filtered open3d point cloud

<a name="open3d_ros_helper.p2p_icp_registration"></a>
#### p2p\_icp\_registration

```python
p2p_icp_registration(source_cloud, target_cloud, n_points=100, threshold=0.02, relative_fitness=1e-10, relative_rmse=1e-8, max_iteration=500, max_correspondence_distance=500)
```

align the source cloud to the target cloud using point-to-point ICP registration algorithm

**Arguments**:

- `source_cloud` _open3d.geometry.PointCloud_ - source open3d point cloud
- `target_cloud` _open3d.geometry.PointCloud_ - target open3d point cloud
  for other parameter, go to http://www.open3d.org/docs/0.9.0/python_api/open3d.registration.registration_icp.html

**Returns**:

- `icp_result` _open3d.registration.RegistrationResult_ - registration result

<a name="open3d_ros_helper.ppf_icp_registration"></a>
#### ppf\_icp\_registration

```python
ppf_icp_registration(source_cloud, target_cloud, n_points=3000, n_iter=100, tolerance=0.05, num_levels=5)
```

align the source cloud to the target cloud using point pair feature (PPF) match

**Arguments**:

- `source_cloud` _open3d.geometry.PointCloud_ - source open3d point cloud
- `target_cloud` _open3d.geometry.PointCloud_ - target open3d point cloud
  for other parameter, go to https://docs.opencv.org/master/dc/d9b/classcv_1_1ppf__match__3d_1_1ICP.html

**Returns**:

- `pose` _np.array_ - 4x4 transformation between source and targe cloud
- `residual` _float_ - the output resistration error


