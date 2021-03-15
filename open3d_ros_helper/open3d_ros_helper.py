import ros_numpy
import open3d
import numpy as np
import tf.transformations as t
import rospy
import copy
import image_geometry
import cv2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Transform, TransformStamped, Vector3
import numpy as np
import numpy.matlib as npm


def pose_to_pq(pose):
    """ convert a ROS PoseS message into position/quaternion np arrays
    Args:
        pose (geometry_msgs/Pose): ROS geometric message to be converted
    Returns:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
    source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
    """
    p = np.array([pose.position.x, pose.position.y, pose.position.z])
    q = np.array([pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w])
    return p, q


def pose_stamped_to_pq(pose_stamped):
    """ convert a ROS PoseStamped message into position/quaternion np arrays
    Args:
        pose_stamped (geometry_msgs/PoseStamped): ROS geometric message to be converted
    Returns:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
    source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
    """
    return pose_to_pq(pose_stamped.pose)


def transform_to_pq(transform):
    """ convert a ROS Transform message into position/quaternion np arrays
    Args:
        transform (geometry_msgs/Transform): ROS geometric message to be converted
    Returns:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
    source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
    """
    p = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
    q = np.array([transform.rotation.x, transform.rotation.y,
                  transform.rotation.z, transform.rotation.w])
    return p, q


def transform_stamped_to_pq(transform_stamped):
    """ convert a ROS TransformStamped message into position/quaternion np arrays
    Args:
        transform_stamped (geometry_msgs/TransformStamped): ROS geometric message to be converted
    Returns:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
    source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
    """
    return transform_to_pq(transform_stamped.transform)


def msg_to_se3(msg):
    """ convert geometric ROS messages to SE(3)
    Args:
        msg (geometry_msgs/Pose, geometry_msgs/PoseStamped, 
        geometry_msgs/Transform, geometry_msgs/TransformStamped): ROS geometric messages to be converted
    Returns:
        se3 (np.array): a 4x4 SE(3) matrix as a numpy array
    source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
    """
    if isinstance(msg, Pose):
        p, q = pose_to_pq(msg)
    elif isinstance(msg, PoseStamped):
        p, q = pose_stamped_to_pq(msg)
    elif isinstance(msg, Transform):
        p, q = transform_to_pq(msg)
    elif isinstance(msg, TransformStamped):
        p, q = transform_stamped_to_pq(msg)
    else:
        raise TypeError("Invalid type for conversion to SE(3)")
    norm = np.linalg.norm(q)
    if np.abs(norm - 1.0) > 1e-3:
        raise ValueError(
            "Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(
                str(q), np.linalg.norm(q)))
    elif np.abs(norm - 1.0) > 1e-6:
        q = q / norm
    se3 = t.quaternion_matrix(q)
    se3[0:3, -1] = p
    return se3


def pq_to_pose_stamped(p, q, source_frame, target_frame, stamp=None):
    """ convert position, quaternion to  geometry_msgs/PoseStamped
    Args:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
        source_frame (string): name of tf source frame
        target_frame (string): name of tf target frame
    Returns:
        pose_stamped (geometry_msgs/PoseStamped): ROS geometric message to be converted of given p and q
    """
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = source_frame
    if stamp is None: stamp = rospy.Time.now() 
    pose_stamped.header.stamp = stamp
    pose_stamped.child_frame_id = target_frame
    pose_stamped.pose = pq_to_pose(p, q)

    return pose_stamped


def pq_to_pose(p, q):
    """ convert position, quaternion to geometry_msgs/Pose
    Args:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
    Returns:
        pose (geometry_msgs/Pose): ROS geometric message to be converted of given p and q
    """
    pose = Pose()
    pose.position.x = p[0]
    pose.position.y = p[1]
    pose.position.z = p[2]
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose

def pq_to_transform(p, q):
    """ convert position, quaternion to geometry_msgs/Transform
    Args:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
    Returns:
        transform (geometry_msgs/Transform): ROS transform of given p and q
    """
    transform = Transform()
    transform.translation.x = p[0]
    transform.translation.y = p[1]
    transform.translation.z = p[2]
    transform.rotation.x = q[0]
    transform.rotation.y = q[1]
    transform.rotation.z = q[2]
    transform.rotation.w = q[3]
    return transform

def pq_to_transform_stamped(p, q, source_frame, target_frame, stamp=None):
    """ convert position, quaternion to geometry_msgs/TransformStamped

    Args:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
        source_frame (string): name of tf source frame
        target_frame (string): name of tf target frame
    Returns:
        transform_stamped (geometry_msgs/TransformStamped): ROS transform_stamped of given p and q
    """

    transform_stamped = TransformStamped()
    transform_stamped.header.frame_id = source_frame
    if stamp is None: stamp = rospy.Time.now() 
    transform_stamped.header.stamp = stamp
    transform_stamped.child_frame_id = target_frame
    transform_stamped.transform = pq_to_transform(p, q)

    return transform_stamped

def se3_to_transform(transform_nparray):
    """ convert 4x4 SE(3) to geometry_msgs/Transform
    Args:
        transform_nparray (np.array): 4x4 SE(3) 
    Returns:
        transform (geometry_msgs/Transform): ROS transform of given SE(3)
    """
    pos = transform_nparray[:3, 3] 
    quat = t.quaternion_from_matrix(transform_nparray)
    transform = pq_to_transform(pos, quat)
    return transform


def se3_to_transform_stamped(transform_nparray, source_frame, target_frame, stamp=None):
    """ convert 4x4 SE(3) to geometry_msgs/TransformStamped
    Args:
        transform_nparray (np.array): 4x4 SE(3) 
        source_frame (string): name of tf source frame
        target_frame (string): name of tf target frame
    Returns:
        transform_stamped (geometry_msgs/TransformStamped): ROS transform_stamped of given SE(3)
    """
    pos = transform_nparray[:3, 3] 
    quat = t.quaternion_from_matrix(transform_nparray)
    if stamp is None: stamp = rospy.Time.now() 
    transform_stamped = pq_to_transform_stamped(pos, quat, source_frame, target_frame, stamp)
    return transform_stamped


def average_q(qs):
    """ calculate the average of quaternions
    Args:
        qs (np.array): multiple quaternion array of shape Nx4
    Returns:
        q_average (np.array): averaged quaternion array
    source codes from https://github.com/christophhagen/averaging-quaternions
    """
    # Number of quaternions to average
    M = qs.shape[0]
    A = npm.zeros(shape=(4,4))
    for i in range(0,M):
        q = qs[i,:]
        # multiply q with its transposed version q' and add A
        A = np.outer(q, q) + A
    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    q_average = np.real(eigenVectors[:,0].A1)
    return q_average


def average_pq(ps, qs):
    """ average the multiple position and quaternion array
    Args:
        ps (np.array): multiple position array of shape Nx3 
        qs (np.array): multiple quaternion array of shape Nx4 
    Returns:
        p_mean (np.array): averaged position array
        q_mean (np.array): averaged quaternion array
    """
    p_average = np.mean(np.asarray(ps), axis=0)
    q_average = average_q(np.asarray(qs))
    return p_average, q_average


convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)

def rospc_to_o3dpc(rospc, remove_nans=False):
    """ covert ros point cloud to open3d point cloud
    Args: 
        rospc (sensor.msg.PointCloud2): ros point cloud message
        remove_nans (bool): if true, ignore the NaN points
    Returns: 
        o3dpc (open3d.geometry.PointCloud): open3d point cloud
    """
    field_names = [field.name for field in rospc.fields]
    is_rgb = 'rgb' in field_names
    cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(rospc)
    if remove_nans:
        mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
        cloud_array = cloud_array[mask]
    if is_rgb:
        cloud_npy = np.zeros(cloud_array.shape + (4,), dtype=np.float)
    else: 
        cloud_npy = np.zeros(cloud_array.shape + (3,), dtype=np.float)
    
    cloud_npy[...,0] = cloud_array['x']
    cloud_npy[...,1] = cloud_array['y']
    cloud_npy[...,2] = cloud_array['z']
    o3dpc = open3d.geometry.PointCloud()

    if len(np.shape(cloud_npy)) == 3:
        cloud_npy = np.reshape(cloud_npy[:, :, :3], [-1, 3], 'F')
    o3dpc.points = open3d.utility.Vector3dVector(cloud_npy[:, :3])

    if is_rgb:
        rgb_npy = cloud_array['rgb']
        rgb_npy.dtype = np.uint32
        r = np.asarray((rgb_npy >> 16) & 255, dtype=np.uint8)
        g = np.asarray((rgb_npy >> 8) & 255, dtype=np.uint8)
        b = np.asarray(rgb_npy & 255, dtype=np.uint8)
        rgb_npy = np.asarray([r, g, b])
        rgb_npy = rgb_npy.astype(np.float)/255
        rgb_npy = np.swapaxes(rgb_npy, 0, 1)
        o3dpc.colors = open3d.utility.Vector3dVector(rgb_npy)
    return o3dpc

BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8

def o3dpc_to_rospc(o3dpc, frame_id=None, stamp=None):
    """ convert open3d point cloud to ros point cloud
    Args:
        o3dpc (open3d.geometry.PointCloud): open3d point cloud
        frame_id (string): frame id of ros point cloud header
        stamp (rospy.Time): time stamp of ros point cloud header
    Returns:
        rospc (sensor.msg.PointCloud2): ros point cloud message
    """

    cloud_npy = np.asarray(copy.deepcopy(o3dpc.points))
    is_color = o3dpc.colors
        

    n_points = len(cloud_npy[:, 0])
    if is_color:
        data = np.zeros(n_points, dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('rgb', np.uint32)
        ])
    else:
        data = np.zeros(n_points, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
            ])
    data['x'] = cloud_npy[:, 0]
    data['y'] = cloud_npy[:, 1]
    data['z'] = cloud_npy[:, 2]
    
    if is_color:
        rgb_npy = np.asarray(copy.deepcopy(o3dpc.colors))
        rgb_npy = np.floor(rgb_npy*255) # nx3 matrix
        rgb_npy = rgb_npy[:, 0] * BIT_MOVE_16 + rgb_npy[:, 1] * BIT_MOVE_8 + rgb_npy[:, 2]  
        rgb_npy = rgb_npy.astype(np.uint32)
        data['rgb'] = rgb_npy

    rospc = ros_numpy.msgify(PointCloud2, data)
    if frame_id is not None:
        rospc.header.frame_id = frame_id

    if stamp is None:
        rospc.header.stamp = rospy.Time.now()
    else:
        rospc.header.stamp = stamp
    rospc.height = 1
    rospc.width = n_points
    rospc.fields = []
    rospc.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
    rospc.fields.append(PointField(
                            name="y",
                            offset=4,
                            datatype=PointField.FLOAT32, count=1))
    rospc.fields.append(PointField(
                            name="z",
                            offset=8,
                            datatype=PointField.FLOAT32, count=1))    

    if is_color:
        rospc.fields.append(PointField(
                        name="rgb",
                        offset=12,
                        datatype=PointField.UINT32, count=1))    
        rospc.point_step = 16
    else:
        rospc.point_step = 12
    
    rospc.is_bigendian = False
    rospc.row_step = rospc.point_step * n_points
    rospc.is_dense = True
    return rospc

def do_transform_point(o3dpc, transform_stamped):
    """ transform a input cloud with respect to the specific frame
        open3d version of tf2_geometry_msgs.do_transform_point
    Args: 
        o3dpc (open3d.geometry.PointCloud): open3d point cloud
        transform_stamped (geometry_msgs.msgs.TransformStamped): transform to be applied 
    Returns:
        o3dpc (open3d.geometry.PointCloud): transformed open3d point cloud
    """
    H = msg_to_se3(transform_stamped)
    o3dpc = copy.deepcopy(o3dpc)
    o3dpc.transform(H)
    return o3dpc

def apply_pass_through_filter(o3dpc, x_range, y_range, z_range):
    """ apply 3D pass through filter to the open3d point cloud
    Args:
        o3dpc (open3d.geometry.PointCloud): open3d point cloud
        x_range (list): list of [x_min, x_maz]
        y_range (list): list of [y_min, y_maz]
        z_range (list): list of [z_min, z_max]
    Returns:
        o3dpc (open3d.geometry.PointCloud): filtered open3d point cloud
    some codes from https://github.com/powersimmani/example_3d_pass_through-filter_guide
    """
    o3dpc = copy.deepcopy(o3dpc)
    cloud_npy = np.asarray(o3dpc.points)
    x_range = np.logical_and(cloud_npy[:, 0] >= x_range[0], cloud_npy[:, 0] <= x_range[1])
    y_range = np.logical_and(cloud_npy[:, 1] >= y_range[0], cloud_npy[:, 1] <= y_range[1])
    z_range = np.logical_and(cloud_npy[:, 2] >= z_range[0], cloud_npy[:, 2] <= z_range[1])
    pass_through_filter = np.logical_and(x_range, np.logical_and(y_range, z_range))
    o3dpc.points = open3d.utility.Vector3dVector(cloud_npy[pass_through_filter])
    
    colors = np.asarray(o3dpc.colors)
    if len(colors) > 0:
        o3dpc.colors = open3d.utility.Vector3dVector(colors[pass_through_filter])
    return o3dpc

def crop_with_2dmask(o3dpc, mask, K=None):
    """ crop open3d point cloud with given 2d binary mask
    Args: 
        o3dpc (open3d.geometry.PointCloud): open3d point cloud
        mask (np.array): binary mask aligned with the point cloud frame shape of [H, W]
        K (np.array): intrinsic matrix of camera shape of (4x4)
        if K is not given, point cloud should be ordered
    Returns:
        o3dpc (open3d.geometry.PointCloud): filtered open3d point cloud
    """
    o3dpc = copy.deepcopy(o3dpc)
    cloud_npy = np.asarray(o3dpc.points)

    if K is None:
        mask = np.resize(mask, cloud_npy.shape[0])
        cloud_npy = cloud_npy[mask!=0]
        o3dpc = open3d.geometry.PointCloud()
        o3dpc.points = open3d.utility.Vector3dVector(cloud_npy)
    else:
        # project 3D points to 2D pixel
        cloud_npy = np.asarray(o3dpc.points)  
        x = cloud_npy[:, 0]
        y = cloud_npy[:, 1]
        z = cloud_npy[:, 2]
        px = np.uint16(x * K[0, 0]/z + K[0, 2])
        py = np.uint16(y * K[1, 1]/z + K[1, 2])
        # filter out the points out of the image
        H, W = mask.shape
        row_indices = np.logical_and(0 <= px, px < W-1)
        col_indices = np.logical_and(0 <= py, py < H-1)
        image_indices = np.logical_and(row_indices, col_indices)
        cloud_npy = cloud_npy[image_indices]
        mask_indices = mask[(py[image_indices], px[image_indices])]
        mask_indices = np.where(mask_indices != 0)[0]
        o3dpc.points = open3d.utility.Vector3dVector(cloud_npy[mask_indices])
    return o3dpc

def p2p_icp_registration(source_cloud, target_cloud, n_points=100, threshold=0.02, \
    relative_fitness=1e-10, relative_rmse=1e-8, max_iteration=500, max_correspondence_distance=500):
    """ align the source cloud to the target cloud using point-to-point ICP registration algorithm
    Args: 
        source_cloud (open3d.geometry.PointCloud): source open3d point cloud
        target_cloud (open3d.geometry.PointCloud): target open3d point cloud
        for other parameter, go to http://www.open3d.org/docs/0.9.0/python_api/open3d.registration.registration_icp.html
    Returns:
        icp_result (open3d.registration.RegistrationResult): registration result
    """
    source_cloud = copy.deepcopy(source_cloud)
    target_cloud = copy.deepcopy(target_cloud)
    n_source_points = np.shape(source_cloud.points)[0]
    n_target_points = np.shape(target_cloud.points)[0]
    n_sample = np.min([n_source_points, n_target_points, n_points])
    source_idxes = np.random.choice(n_source_points, n_sample, replace=False)
    target_idxes = np.random.choice(n_target_points, n_sample, replace=False)
    source_cloud = source_cloud.select_down_sample(source_idxes)
    target_cloud = target_cloud.select_down_sample(target_idxes)
    trans_init = np.eye(4)
    evaluation = open3d.registration.evaluate_registration(source_cloud, target_cloud, threshold, trans_init)
    icp_result = open3d.registration.registration_icp(
        source = source_cloud, target = target_cloud, max_correspondence_distance=max_correspondence_distance, # unit in millimeter
        init = np.eye(4),  
        estimation_method = open3d.registration.TransformationEstimationPointToPoint(), 
        criteria = open3d.registration.ICPConvergenceCriteria(
                                            relative_fitness=relative_fitness,
                                            relative_rmse=relative_rmse,
                                            max_iteration=max_iteration))                                               
    return icp_result, evaluation
            
def ppf_icp_registration(source_cloud, target_cloud, n_points=3000, n_iter=100, tolerance=0.001, num_levels=5, scale=0.001):
    """ align the source cloud to the target cloud using point pair feature (PPF) match
    Args: 
        source_cloud (open3d.geometry.PointCloud): source open3d point cloud
        target_cloud (open3d.geometry.PointCloud): target open3d point cloud
        for other parameter, go to https://docs.opencv.org/master/dc/d9b/classcv_1_1ppf__match__3d_1_1ICP.html
    Returns:
        pose (np.array): 4x4 transformation between source and targe cloud
        residual (float): the output resistration error
    """
    source_cloud = copy.deepcopy(source_cloud)
    source_cloud = source_cloud.voxel_down_sample(scale)
    target_cloud = copy.deepcopy(target_cloud)
    n_source_points = np.shape(source_cloud.points)[0]
    n_target_points = np.shape(target_cloud.points)[0]
    n_sample = np.min([n_source_points, n_target_points, n_points])
    if n_sample == 0:
        return None, 10000
    if n_source_points > n_points:
        source_idxes = np.random.choice(n_source_points, n_sample, replace=False)
        source_cloud = source_cloud.select_down_sample(source_idxes)
    if n_target_points > n_points:
        target_idxes = np.random.choice(n_target_points, n_sample, replace=False)
        target_cloud = target_cloud.select_down_sample(target_idxes)
    target_cloud.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    source_np_cloud = np.concatenate([np.asarray(source_cloud.points), np.asarray(source_cloud.normals)], axis=1).astype(np.float32)
    target_np_cloud = np.concatenate([np.asarray(target_cloud.points), np.asarray(target_cloud.normals)], axis=1).astype(np.float32)
    icp_fnc = cv2.ppf_match_3d_ICP(n_iter, tolerence=tolerance, rejectionScale=2.5, numLevels=num_levels) 
    try:
        retval, residual, pose = icp_fnc.registerModelToScene(source_np_cloud, target_np_cloud)
    except:
        return None, 10000
    else:
        return pose, residual


