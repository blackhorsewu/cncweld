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

def callback_roscloud(ros_cloud):
    global received_ros_cloud

    received_ros_cloud = ros_cloud

def normalize_feature(feature_value_list):
    
    normalized_feature_value_list = (feature_value_list-feature_value_list.min())/(feature_value_list.max()-feature_value_list.min())

    return np.array(normalized_feature_value_list)

# find feature value list
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
        # feature_value = (255.0-n_list[index,2])/255.0
        # feature_value_list.append(feature_value)

    return np.array(feature_value_list)

def detect_groove_workflow(pcd):

    original_pcd = pcd

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
    feature_value_list = find_feature_value()
    normalized_feature_value_list = normalize_feature(feature_value_list)

    # 4. Delete low value points and cluster
    delete_points = int(pc_number*delete_percentage)
    pcd_selected = pcd.select_down_sample(
        np.argsort(normalized_feature_value_list[delete_points:])
    )

    groove = cluster_groove_from_point_cloud(pcd_selected, voxel_size)

    rviz_cloud = convertCloudFromOpen3dToRos(groove)
    pub_transformed.publish(rviz_cloud)

if __name__ == "__main__":

    rospy.init_node('cnc_weld', anonymous=True)

    global received_ros_cloud

    rospy.Subscriber('/d435i/depth/color/points', PointCloud2, callback_roscloud, queue_size=1)

    while not rospy.is_shutdown():

        if not received_ros_cloud is None:
            print("\n ************* Start groove detection *************")
            received_open3d_cloud = convertCloudFromRosToOpen3d(received_ros_cloud)
            detect_groove_workflow(received_open3d_cloud)




