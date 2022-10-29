#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
  Chinese National Engineering Research Centre for Steel Structure
  (Hong Kong Branch)
  Hong Kong Polytechnic University
  Author: Victor W H Wu
  Date: 27 October, 2022.

  This script tries to detect groove in the workpiece.

  It requires:
  rospy, 
  numpy, 
  open3d, 
  math

'''

import numpy as np
import open3d as o3d
# from cncweld_core.src.detect.script.seam_detection_line import cluster_groove_from_point_cloud, convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d, detect_groove_workflow
import rospy
import math

from ctypes import * # convert float to uint32

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

## *********************************************************************** ##

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
# requires sensor_msgs.msg / PointCloud2, PointField
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
# requires * of ctypes
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

## *********************************************************************** ##

def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="d435i_depth_optical_frame"):
    # Set "header"
    header = Header() # from std_msgs.msg import Header
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors: # XYZ only
        fields=FIELDS_XYZ
        cloud_data=points
    else: # XYZ + RGB
        fields=FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
        colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
        cloud_data=np.c_[points, colors]
    
    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)

def convertCloudFromRosToOpen3d(ros_cloud):
    
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud

## *********************************************************************** ##

def callback_roscloud(ros_cloud):
    global received_ros_cloud

    received_ros_cloud = ros_cloud

def normalize_feature(feature_value_list):
    
    normalized_feature_value_list = (feature_value_list-feature_value_list.min())/(feature_value_list.max()-feature_value_list.min())

    return np.array(normalized_feature_value_list)

# find feature value of each point of the point cloud and put them into a list
def find_feature_value(pcd, voxel_size):

    # Build a KD (k-dimensional) Tree for Flann
    # Fast Library for Approximate Nearest Neighbor
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)

    # Treat pcd.points as an numpy array of n points by m attributes of a point
    # The first dimension, shape[0], of this array is the number of points in this point cloud.
    pc_number = np.asarray(pcd.points).shape[0]

    feature_value_list = []
    
    # n_list is an array of normals of all the points
    n_list = np.asarray(pcd.normals)
    # n_list = np.asarray(pcd.colors)

    # a // b = integer quotient of a divided by b
    # so neighbor (number of neighbors) whichever is smaller of 30 or the quotient 
    # of dividing the number of points by 100
    neighbour = min(pc_number//100, 30)
    
    # for every point of the point cloud
    for index in range(pc_number):
        
        # Search the k nearest neighbour for each point of the point cloud
        # The pcd.points[index] is the point to find the nearest neighbour for
        # neighbour, found above, is the neighbours to be searched
        [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[index], neighbour)

        # n_list, computed above, is an array of normals of every point
        # vector is then a vector with its components the arithmetic mean of every
        # element of all the k neighbours of that point
        vector = np.mean(n_list[idx, :], axis=0)
        
        # the bigger the feature value, meaning the normal of that point is more 
        # different from its neighbours
        feature_value = np.linalg.norm(
            vector - n_list[index, :] * np.dot(vector,n_list[index, :]) / np.linalg.norm(n_list[index, :]))
        feature_value_list.append(feature_value)

    return np.array(feature_value_list)

def cluster_groove_from_point_cloud(pcd, voxel_size, verbose=False):

    global neighbor

    # eps - the maximum distance between neighbours in a cluster,
    # at least min_points to form a cluster.
    # returns an array of labels, each label is a number. 
    # labels of the same cluster have their labels the same number.
    # if this number is -1, that is this cluster is noise.
    labels = np.array(pcd.cluster_dbscan(eps=0.005, min_points=3, print_progress=True))

    # np.unique returns unique labels, label_counts is an array of number of that label
    label, label_counts = np.unique(labels, return_counts=True)

    # Find the largest cluster
    ## [-1] is the last element of the array, minus means counting backward.
    ## So, after sorting ascending the labels with the cluster with largest number of
    ## members at the end. That is the largest cluster.
    label_number = label[np.argsort(label_counts)[-1]]

    if label_number == -1:
        if label.shape[0]>1:
            label_number = label[np.argsort(label_counts)[-2]]
        elif label.shape[0]==1:
            # sys.exit("can not find a valid groove cluster")
            print("can not find a valid groove cluster")
    
    # Pick the points belong to the largest cluster
    groove_index = np.where(labels == label_number)
    groove = pcd.select_down_sample(groove_index[0])

    return groove

def detect_groove_workflow(pcd):

    original_pcd = pcd

    global delete_percentage

    # 1. Down sample the point cloud
    ## a. Define a bounding box for cropping
    bbox = o3d.geometry.AxisAlignedBoundingBox(
        min_bound = (-0.5, -0.5, 0), # x right, y down, z forward; for the camera
        max_bound = (0.5, 0.5, 0.5)  # 1m x 1m plane with 0.5m depth
    )

    ## b. Define voxel size
    voxel_size = 0.001 # 1mm cube for each voxel

    pcd = pcd.voxel_down_sample(voxel_size = voxel_size)
    pcd = pcd.crop(bbox)

    ### it was remove_none_finite_points in Open3D version 0.8.0... but
    ### it is  remove_non_finite_points  in Open3D version 0.15.1...
    pcd.remove_non_finite_points()

    ## c. Count the number of points afterwards
    pc_number = np.asarray(pcd.points).shape[0]
    rospy.loginfo("Total number of points {}".format(pc_number))

    # 2. Estimate normal toward camera location and normalize it.
    pcd.estimate_normals(
        search_param = o3d.geometry.KDTreeSearchParamHybrid(
            radius = 0.01,
            max_nn = 30
        )
    )
    pcd.normalize_normals()
    pcd.orient_normals_towards_camera_location(camera_location = [0., 0., 0.])

    # 3. Use different geometry features to find groove
    feature_value_list = find_feature_value(pcd, voxel_size)
    normalized_feature_value_list = normalize_feature(feature_value_list)

    # 4. Delete low value points and cluster
    delete_points = int(pc_number * delete_percentage)

    rviz_cloud = convertCloudFromOpen3dToRos(pcd)
    pub_pc.publish(rviz_cloud)

    pcd_selected = pcd.select_down_sample(
        ## np.argsort performs an indirect sort
        ## and returns an array of indices of the same shape
        ## that index data along the sorting axis
        ## in ascending order by default. So the smaller value first
        ## and the largest value at the end
        np.argsort(normalized_feature_value_list)[delete_points:]
        ## therefore this is a list of indices of the point cloud
        ## with the top 5 percent feature value
    )

    groove = cluster_groove_from_point_cloud(pcd_selected, voxel_size)

    rviz_cloud = convertCloudFromOpen3dToRos(groove)
    pub_transformed.publish(rviz_cloud)

if __name__ == "__main__":

    rospy.init_node('cnc_weld', anonymous=True)

    global received_ros_cloud, delete_percentage

    delete_percentage = 0.95

    received_ros_cloud = None

    # Setup subscriber
    rospy.Subscriber('/d435i/depth/color/points', PointCloud2, callback_roscloud, queue_size=1)

    # Setup publisher
    pub_transformed = rospy.Publisher("transformed", PointCloud2, queue_size=1)
    pub_pc = rospy.Publisher("downsampled_points", PointCloud2, queue_size=1)

    while not rospy.is_shutdown():

        if not received_ros_cloud is None:
            print("\n ************* Start groove detection *************")
            received_open3d_cloud = convertCloudFromRosToOpen3d(received_ros_cloud)
            detect_groove_workflow(received_open3d_cloud)




