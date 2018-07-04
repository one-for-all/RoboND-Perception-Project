#!/usr/bin/env python

# Import modules
import math
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

from pcl_helper import *


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict


# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Exercise-2 TODOs:
    # TODO: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)

    # TODO: Statistical Outlier Filtering
    noise_filter = pcl_data.make_statistical_outlier_filter()
    noise_filter.set_mean_k(50)
    x = 1.0
    noise_filter.set_std_dev_mul_thresh(x)
    cloud_filtered = noise_filter.filter()
    pcl_noise_filtered_pub.publish(pcl_to_ros(cloud_filtered))

    # TODO: Voxel Grid Downsampling
    vox = cloud_filtered.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
    pcl_downsampled_pub.publish(pcl_to_ros(cloud_filtered))

    # TODO: PassThrough Filter
    USE_PASSTHROUGH = False
    if USE_PASSTHROUGH:
        # about z-axis
        passthrough = cloud_filtered.make_passthrough_filter()
        filter_axis = 'z'
        passthrough.set_filter_field_name(filter_axis)
        axis_min = 0.6
        axis_max = 1.1
        passthrough.set_filter_limits(axis_min, axis_max)
        cloud_filtered = passthrough.filter()
        passthrough = cloud_filtered.make_passthrough_filter()
        # about y-axis
        filter_axis = 'y'
        passthrough.set_filter_field_name(filter_axis)
        axis_min = -0.42
        axis_max = 0.42
        passthrough.set_filter_limits(axis_min, axis_max)
        cloud_filtered = passthrough.filter()
        pcl_passthrough_filtered_pub.publish(pcl_to_ros(cloud_filtered))

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.02
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    pcl_table_pub.publish(pcl_to_ros(cloud_table))
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    pcl_objects_pub.publish(pcl_to_ros(cloud_objects))

    # Filter more planes
    for i in range(2):
        seg = cloud_objects.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        max_distance = 0.02
        seg.set_distance_threshold(max_distance)
        inliers, coefficients = seg.segment()
        print("inliers: {}".format(len(inliers)))
        if len(inliers) < 500:
            break
        else:
            cloud_objects = cloud_objects.extract(inliers, negative=True)

    pcl_objects_2_pub.publish(pcl_to_ros(cloud_objects))

    # TODO: Statistical Outlier Filtering again after getting objects
    noise_filter = cloud_objects.make_statistical_outlier_filter()
    noise_filter.set_mean_k(20)
    x = 0.9
    noise_filter.set_std_dev_mul_thresh(x)
    cloud_objects = noise_filter.filter()
    pcl_noise_filtered_objects_pub.publish(pcl_to_ros(cloud_objects))

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(30)
    ec.set_MaxClusterSize(5000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, index in enumerate(indices):
            color_cluster_point_list.append([white_cloud[index][0],
                                             white_cloud[index][1],
                                             white_cloud[index][2],
                                             rgb_to_float(cluster_color[j])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages

    # TODO: Publish ROS messages
    pcl_cluster_pub.publish(pcl_to_ros(cluster_cloud))


    # Exercise-3 TODOs:
    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects_list = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += 0.4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects_list.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # detected_objects_pub.publish(detected_objects_list)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    # try:
    #     pr2_mover(detected_objects_list)
    # except rospy.ROSInterruptException:
    #     pass


# function to load parameters and request PickPlace service
def pr2_mover(detected_objects_list):

    # TODO: Initialize variables
    labels = []
    centroids = []
    yaml_detected_objects_list = []

    # Initialize message variables
    test_scene_num = Int32()
    object_name = String()
    which_arm = String()
    pick_pose = Pose()
    place_pose = Pose()

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    object_list_id = int(rospy.get_param('/object_list_num')[0]['num'])
    dropbox_param = rospy.get_param('/dropbox')

    # Set box group and side correspondence
    box_side_2_group = {}
    box_group_2_side = {}
    for dropbox in dropbox_param:
        box_side_2_group[dropbox['name']] = dropbox['group']
        box_group_2_side[dropbox['group']] = dropbox['name']

    # TODO: Parse parameters into individual variables
    test_scene_num.data = object_list_id

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for object_param in object_list_param:
        for index, detected_object in enumerate(detected_objects_list):
            if object_param['name'] == detected_object.label:
                # Remove object from list to prevent repetition
                detected_objects_list = detected_objects_list[:index] + detected_objects_list[index+1:]

                object_name.data = object_param['name']

                # TODO: Assign the arm to be used for pick_place
                which_arm.data = box_group_2_side[object_param['group']]

                # TODO: Get the PointCloud for a given object and obtain it's centroid
                # Find centroid of detected_object and assign to pick_pose
                points_arr = ros_to_pcl(detected_object.cloud).to_array()
                centroid = np.mean(points_arr, axis=0)[:3]
                centroid = centroid.tolist()
                pick_pose.position.x = centroid[0]
                pick_pose.position.y = centroid[1]
                pick_pose.position.z = centroid[2]

                # TODO: Create 'place_pose' for the object
                for dropbox in dropbox_param:
                    dropbox_position = dropbox['position']
                    if dropbox['group'] == object_param['group']:
                        place_pose.position.x = dropbox_position[0]
                        place_pose.position.y = dropbox_position[1]
                        place_pose.position.z = dropbox_position[2]

                # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
                yaml_dict = make_yaml_dict(test_scene_num, which_arm, object_name, pick_pose, place_pose)
                yaml_detected_objects_list.append(yaml_dict)

                # Wait for 'pick_place_routine' service to come up
                rospy.wait_for_service('pick_place_routine')

                # try:
                #     pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
                #
                #     # TODO: Insert your message variables to be sent as a service request
                #     resp = pick_place_routine(test_scene_num, object_name, which_arm, pick_pose, place_pose)
                #
                #     print ("Response: ", resp.success)
                #
                # except rospy.ServiceException, e:
                #     print "Service call failed: %s"%e

                break

    # TODO: Output your request parameters into output yaml file
    send_to_yaml('output_{}.yaml'.format(test_scene_num.data), yaml_detected_objects_list)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_noise_filtered_pub = rospy.Publisher("/pcl_noise_filtered", PointCloud2, queue_size=1)
    pcl_downsampled_pub = rospy.Publisher("/pcl_downsampled", PointCloud2, queue_size=1)
    pcl_passthrough_filtered_pub = rospy.Publisher("/pcl_passthrough_filtered", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_objects_2_pub = rospy.Publisher("/pcl_objects_2", PointCloud2, queue_size=1)
    pcl_noise_filtered_objects_pub = rospy.Publisher("/pcl_noise_filtered_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # Motion Control publishers
    world_joint_pub = rospy.Publisher("/pr2/world_joint_controller/command", Float64, queue_size=1)
    obstacles_pub = rospy.Publisher("/pr2/3D_map/points", PointCloud2, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    rate = rospy.Rate(1)

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        # rospy.spin()
        world_joint_pub.publish(math.pi/2)
        rate.sleep()
