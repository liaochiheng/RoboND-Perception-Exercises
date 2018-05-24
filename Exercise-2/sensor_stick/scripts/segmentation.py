#!/usr/bin/env python

# Import modules
import rospy
import pcl
import numpy as np
import ctypes
import struct

from sensor_msgs.msg import PointCloud2
from pcl_helper import *

# Define functions as required
def vox_filt( cloud, LEAF_SIZE = 0.01 ):
    vox = cloud.make_voxel_grid_filter()
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    return vox.filter()

def passthrough_filt( cloud, filter_axis = 'z', axis_min = 0.77, axis_max = 1.1 ):
    passthrough = cloud.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

def seg_plane( cloud, max_distance = 0.01 ):
    seg = cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()
    return inliers, coefficients

def euclidean_cluster( white_cloud ):
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance( 0.05 )
    ec.set_MinClusterSize( 10 )
    ec.set_MaxClusterSize( 2500 )
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    return ec.Extract()

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data
    data = ros_to_pcl( pcl_msg )

    #  Voxel Grid Downsampling
    cloud = vox_filt( data )

    # PassThrough Filter
    cloud = passthrough_filt( cloud )

    # RANSAC Plane Segmentation
    inliers, coefficients = seg_plane( cloud )

    # Extract inliers and outliers
    cloud_table = cloud.extract( inliers, negative = False )
    cloud_objects = cloud.extract( inliers, negative = True )

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ( cloud_objects )
    cluster_indices = euclidean_cluster( white_cloud )

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list( len( cluster_indices ) )

    color_cluster_point_list = []

    for j, indices in enumerate( cluster_indices ):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append( [ white_cloud[ indice ][ 0 ],
                                               white_cloud[ indice ][ 1 ],
                                               white_cloud[ indice ][ 2 ],
                                               rgb_to_float( cluster_color[ j ] ) ] )

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list( color_cluster_point_list )

    ros_cluster_cloud = pcl_to_ros( cluster_cloud )

    # Convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros( cloud_table )
    ros_cloud_objects = pcl_to_ros( cloud_objects )

    # Publish ROS messages
    pcl_table_pub.publish( ros_cloud_table )
    pcl_objects_pub.publish( ros_cloud_objects )

    pcl_cluster_pub.publish( ros_cluster_cloud )


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node( 'clustering', anonymous = True )

    # Create Subscribers
    pcl_sub = rospy.Subscriber( "/sensor_stick/point_cloud", PointCloud2, pcl_callback, queue_size = 1 )

    # Create Publishers
    pcl_table_pub = rospy.Publisher( "/pcl_table", PointCloud2, queue_size = 1 )
    pcl_objects_pub = rospy.Publisher( "/pcl_objects", PointCloud2, queue_size = 1 )

    pcl_cluster_pub = rospy.Publisher( "/pcl_cluster", PointCloud2, queue_size = 1 )

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
