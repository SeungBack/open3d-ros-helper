import ros_numpy
import open3d
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import tf.transformations as t
import rospy
import copy
import image_geometry
import cv2

def convert_ros_to_o3d(ros_msg, remove_nans=False):

    cloud_npy = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(ros_msg, remove_nans)
    cloud_o3d = open3d.geometry.PointCloud()
    cloud_o3d.points = open3d.utility.Vector3dVector(cloud_npy)
    return cloud_o3d

def convert_o3d_to_ros(cloud_o3d, frame_id=None):

    cloud_npy = np.asarray(copy.deepcopy(cloud_o3d.points))
    n_points = len(cloud_npy[:, 0])
    data = np.zeros(n_points, dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32)
        ])
    data['x'] = cloud_npy[:, 0]
    data['y'] = cloud_npy[:, 1]
    data['z'] = cloud_npy[:, 2]

    ros_msg = ros_numpy.msgify(PointCloud2, data)
    
    if frame_id is not None:
        ros_msg.header.frame_id = frame_id

    ros_msg.header.stamp = rospy.Time.now()
    ros_msg.height = 1
    ros_msg.width = n_points
    ros_msg.fields = []
    ros_msg.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="y",
                            offset=4,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="z",
                            offset=8,
                            datatype=PointField.FLOAT32, count=1))    
    ros_msg.is_bigendian = False
    ros_msg.point_step = 12
    ros_msg.row_step = ros_msg.point_step * n_points
    ros_msg.is_dense = True
    return ros_msg


def do_transform_o3d_cloud(cloud_o3d, transformStamped):

    H = t.quaternion_matrix([
        transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, 
        transformStamped.transform.rotation.z, transformStamped.transform.rotation.w])
    H[:3, 3] = [transformStamped.transform.translation.x, 
                transformStamped.transform.translation.y, 
                transformStamped.transform.translation.z]
    cloud_o3d = copy.deepcopy(cloud_o3d)
    cloud_o3d.transform(H)
    return cloud_o3d

def o3d_cloud_pass_through_filter(cloud_o3d, ROI):
    cloud_o3d = copy.deepcopy(cloud_o3d)
    cloud_npy = np.asarray(cloud_o3d.points)
    x_range = np.logical_and(cloud_npy[:, 0] >= ROI["x"][0], cloud_npy[:, 0] <= ROI["x"][1])
    y_range = np.logical_and(cloud_npy[:, 1] >= ROI["y"][0], cloud_npy[:, 1] <= ROI["y"][1])
    z_range = np.logical_and(cloud_npy[:, 2] >= ROI["z"][0], cloud_npy[:, 2] <= ROI["z"][1])
    pass_through_filter = np.logical_and(x_range, np.logical_and(y_range, z_range))
    cloud_o3d.points = open3d.utility.Vector3dVector(cloud_npy[pass_through_filter])
    return cloud_o3d

def convert_roi_to_3dbbox(ROI):
    bbox3d = []
    for x in ROI['x']:
        for y in ROI['y']:
            for z in ROI['z']:
                bbox3d.append([x, y, z])
    return np.array(bbox3d).astype("float64")


def crop_o3d_cloud_with_mask(cloud_o3d, mask, remove_nans=False, camera_info=None):

    """ 
        if camera_info is not given, point cloud should be ordered and it will be cropped according to the mask pixel-wisely.
    """
    cloud_o3d = copy.deepcopy(cloud_o3d)
    cloud_npy = np.asarray(cloud_o3d.points)

    if camera_info is None:
        mask = np.resize(mask, cloud_npy.shape[0])
        cloud_npy = cloud_npy[mask!=0]
        cloud_o3d = open3d.geometry.PointCloud()
        cloud_o3d.points = open3d.utility.Vector3dVector(cloud_npy)
    else:
        camera_model = image_geometry.PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)
        K = camera_model.intrinsicMatrix()
        
        # mask_rect = np.zeros_like(mask)
        # camera_model.rectifyImage(mask, mask_rect)
        # mask = mask_rect

        # project 3D points to 2D pixel
        cloud_npy = np.asarray(cloud_o3d.points)  
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
        mask_indices = np.where(mask_indices != 0)
        cloud_o3d.points = open3d.utility.Vector3dVector(cloud_npy[mask_indices])
    return cloud_o3d

def icp_refinement(source_cloud, target_cloud, n_points=100, threshold=0.02, \
    relative_fitness=1e-10, relative_rmse=1e-8, max_iteration=500, max_correspondence_distance=500):
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
            
def icp_refinement_with_ppf_match(source_cloud, target_cloud, n_points=3000, n_iter=100, tolerance=0.05, num_levels=5):

    source_cloud = copy.deepcopy(source_cloud)
    source_cloud = source_cloud.voxel_down_sample(0.003)
    target_cloud = copy.deepcopy(target_cloud)
    n_source_points = np.shape(source_cloud.points)[0]
    n_target_points = np.shape(target_cloud.points)[0]
    n_sample = np.min([n_source_points, n_target_points, n_points])
    if n_sample == 0:
        return None, 999999999
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
    retval, residual, pose = icp_fnc.registerModelToScene(source_np_cloud, target_np_cloud)

    return pose, residual


