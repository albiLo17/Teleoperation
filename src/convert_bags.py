#!/usr/bin/env python

import numpy as np
import sys
import rosbag
import glob
from cv_bridge import CvBridge
import rospy
import pickle

if __name__ == '__main__':


    bag_path = '/home/alberta/franka_ws/bags/coop/'

    bag_paths = glob.glob('/home/alberta/franka_ws/bags/coop/*.bag')
    # bag_paths = ['/home/alberta/franka_ws/bags/coop/39.bag']

    for path in bag_paths:

        bag_name = path.split('/')[-1]

        print('converting bag: ' + bag_name)

        bag = rosbag.Bag(bag_path + bag_name)
        bridge = CvBridge()
        markers = []
        times = []

        # for topic, msg, t in bag.read_messages(topics=['/ar_pose_marker']):
        #     times.append(t.to_sec())
        #     mrk = []
        #     temp = np.zeros((2, 7))
        #     for marker in msg.markers:
        #         # position
        #         temp[marker.id][0] = marker.pose.pose.position.x
        #         temp[marker.id][1] = marker.pose.pose.position.y
        #         temp[marker.id][2] = marker.pose.pose.position.z
        #         # orientation
        #         temp[marker.id][3] = marker.pose.pose.orientation.x
        #         temp[marker.id][4] = marker.pose.pose.orientation.y
        #         temp[marker.id][5] = marker.pose.pose.orientation.z
        #         temp[marker.id][6] = marker.pose.pose.orientation.w
        #         mrk.append(temp)
        #
        #     markers.append(mrk)
        name = bag_name.split(".")[0]
        # np.save(name + '_markers.npy', markers)

        # imgs = np.array(images)

        # Save images as ndarray
        # final_path = bag_path + name + '_markers.npy'
        # final_time_path = bag_path + 'time_' + name + '_markers.npy'
        # # final_path = "/media/alberta/ALBI/GarmentMaterialClassification5Classes/bags2/data/"  + materials[int(material)] + "/images_"+ type_exp + "_" + num_material + "_" + num_experiment + ".npy"
        # rospy.loginfo(final_path)
        # np.save(final_path, imgs)
        # np.save(final_time_path, times)

        # marker_file = open(final_path, 'wb')
        # pickle.dump(markers, marker_file)
        # np.save(final_path, markers)
        # np.save(final_time_path, times)

        # sensor2 = {'force': {'x': [], 'y': [], 'z': []}, 'torque': {'x': [], 'y': [], 'z': []}}
        # times = []
        #
        # for topic, msg, t in bag.read_messages(topics=['/optoforce_node/wrench_UCE0B232']):
        #     times.append(t.to_sec())
        #     sensor2['torque']['x'].append(msg.wrench.torque.x)
        #     sensor2['force']['x'].append(msg.wrench.force.x)
        #     sensor2['torque']['y'].append(msg.wrench.torque.y)
        #     sensor2['force']['y'].append(msg.wrench.force.y)
        #     sensor2['torque']['z'].append(msg.wrench.torque.z)
        #     sensor2['force']['z'].append(msg.wrench.force.z)
        #
        #
        # final_path = bag_path + '/forces/2/'+ bag_name.split('.')[0] +'.pkl'
        # final_time_path = bag_path + '/forces/2/time_'+ bag_name.split('.')[0] +'.npy'
        # sensor2_file = open(final_path, 'wb')
        # pickle.dump(sensor2, sensor2_file)
        # np.save(final_time_path, times)

        # sensor9 = {'force': {'x': [], 'y': [], 'z': []}, 'torque': {'x': [], 'y': [], 'z': []}}
        # times = []
        #
        # for topic, msg, t in bag.read_messages(topics=['/optoforce_node/wrench_UCE0B239']):
        #     times.append(msg.header.stamp.secs)
        #     sensor9['torque']['x'].append(msg.wrench.torque.x)
        #     sensor9['force']['x'].append(msg.wrench.force.x)
        #     sensor9['torque']['y'].append(msg.wrench.torque.y)
        #     sensor9['force']['y'].append(msg.wrench.force.y)
        #     sensor9['torque']['z'].append(msg.wrench.torque.z)
        #     sensor9['force']['z'].append(msg.wrench.force.z)
        #
        # final_path = bag_path + '/forces/9/' + bag_name.split('.')[0] + '.pkl'
        # final_time_path = bag_path + '/forces/9/time_' + bag_name.split('.')[0] + '.npy'
        # sensor9_file = open(final_path, 'wb')
        # pickle.dump(sensor9, sensor9_file)
        # np.save(final_time_path, times)

        joint_pose = []
        times = []
        times_ros = []

        #for topic, msg, t in bag.read_messages(topics=['/franka_state_controller/joint_state']):
        for topic, msg, t in bag.read_messages(topics='/franka_state_controller/joint_states'):
            times_ros.append(t.to_sec())
            #print(t.to_sec())
            times.append(msg.header.stamp.secs)
            joint_pose.append(msg.position)

        final_path = bag_path  + name + '_joints.npy'
        final_time_path = bag_path  + 'time_' + name + '_joints.npy'
        final_time_ros = bag_path + 'time_ros_' + name + '_joints.npy'
        np.save(final_path, joint_pose)
        np.save(final_time_path, times)

        # time_file = open(final_time_ros, 'wb')
        # pickle.dump(final_time_ros, time_file)
        np.save(final_time_ros, times_ros)

        control = []
        times = []

        # for topic, msg, t in bag.read_messages(topics=['/franka_state_controller/joint_state']):
        for topic, msg, t in bag.read_messages(topics='/kth_cartesian_velocity_effort_interface_controller/panda/equilibrium_pose'):
            times.append(t.to_sec())
            temp = np.zeros(7)

            temp[0] = msg.linear.x
            temp[1] = msg.linear.y
            temp[2] = msg.linear.z
            temp[3] = msg.angular.x
            temp[4] = msg.angular.y
            temp[5] = msg.angular.z

            control.append(temp)

        final_path = bag_path + name + '_control.npy'
        final_time_path = bag_path + 'time_' + name + '_control.npy'
        np.save(final_path, joint_pose)
        np.save(final_time_path, times)


