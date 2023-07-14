#!/usr/bin/env python3


from oculus_reader.reader import OculusReader
import numpy as np
from scipy.linalg import logm
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Twist, WrenchStamped
import rospy
import time
from controller_manager_msgs.srv import *
from collections import deque

data_buffer = deque([], maxlen=100)

# ns = '/panda'
ns = ''
# pub = rospy.Publisher('/panda/kth_cartesian_velocity_effort_interface_controller/panda/equilibrium_pose', Twist)
pub = rospy.Publisher(ns + '/kth_cartesian_velocity_effort_interface_controller/panda/equilibrium_pose', Twist, queue_size=10)
pub_pose = rospy.Publisher(ns + '/kth_joint_pose_effort_interface_controller/panda/equilibrium_pose', PoseStamped, queue_size=10)



def send_initial_pose():
    global goal
    goal = PoseStamped()

    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"  # TODO: check this frame ID

    initial = [-0.006448649147283548, -0.18572620676790708, 0.056528580207321995, -1.6648124255179138, 0.013624473211176689, 1.4953778528546542, 0.917335578738194]
    # if self.robot == 'stein':
    #     initial = self.stein_init_pose

    goal.pose.position.x = initial[0]
    goal.pose.position.y = initial[1]
    goal.pose.position.z = initial[2]

    goal.pose.orientation.x = initial[3]
    goal.pose.orientation.y = initial[4]
    goal.pose.orientation.z = initial[5]
    goal.pose.orientation.w = initial[6]

    pub_pose.publish(goal)


def main():
    rospy.init_node('teleop')

    # controller services
    rospy.wait_for_service(ns +"/controller_manager/load_controller")
    s_load_controller = rospy.ServiceProxy(ns + '/controller_manager/load_controller', LoadController)

    # # Load velocity controller
    # name = "kth_cartesian_velocity_effort_interface_controller"
    # resp = s_load_controller.call(LoadControllerRequest(name))


    oculus_reader = OculusReader()

    relative_pos_t = np.zeros(3)
    relative_pos_t1 = np.zeros(3)

    relative_R_t = np.zeros((3, 3))
    relative_R_t1 = np.zeros((3, 3))

    MIN_DISCRETIZATION = 0.0000001 #0.0001
    MAX_ABS_VEL = 0.1
    SCALE_VEL = 0.8   # 1.0 #0.5

    MIN_ROT_DISCRETIZATION = 0.0000001
    MAX_ABS_ROT = 0.2
    SCALE_ROT = 1. #0.4


    controller_ready = False
    while not controller_ready:
        raw_data = oculus_reader.get_transformations_and_buttons()
        if raw_data[0]:
            relative_pos_t = raw_data[0]['r'][:3, 3]
            # relative_rot_t = Rotation.from_matrix(raw_data[0]['r'][:3, :3]).as_rotvec() + np.pi
            relative_R_t = raw_data[0]['r'][:3, :3]
            time_stamp_t = time.time()
            if raw_data[1]['RTr']:
                controller_ready = True
        else:
            print("no sensible data from right controller")
            time.sleep(0.1)

    print("start connection with the robot")

    counter = 0
    angular_vel = np.zeros(3)
    gripper_closed = False
    rate = rospy.Rate(100)      # TODO: might change

    joint_controller = ['kth_joint_pose_effort_interface_controller']
    velocity_controller = ['kth_cartesian_velocity_effort_interface_controller']

    try:
        switch_controller = rospy.ServiceProxy(ns + '/controller_manager/switch_controller', SwitchController)
        ret = switch_controller(velocity_controller, joint_controller, 2, False, 0.0)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    readinput = True

    try:

        while not (rospy.is_shutdown()):
            # if readinput:
            #     r_str = input()
            # if (r_str == '\x03'):
            #     break
            #
            # if r_str == 'init':
            #     print('*** sending to initial position ***')
            #     try:
            #         switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            #         ret = switch_controller(joint_controller, velocity_controller, 2, False, 0.0)
            #         send_initial_pose()
            #     except rospy.ServiceException as e:
            #         print("Service call failed: %s" % e)
            #
            # if r_str == 'go':


            msg = Twist()

            raw_data = oculus_reader.get_transformations_and_buttons()
            relative_pos_t1 += raw_data[0]['r'][:3, 3]
            # relative_rot_t1 = Rotation.from_matrix(raw_data[0]['r'][:3, :3]).as_rotvec() + np.pi
            relative_R_t1 = raw_data[0]['r'][:3, :3]
            time_stamp_t1 = time.time()
            # print(relative_rot_t1)
            # time.sleep(1.)
            counter += 1

            if not raw_data[0] or raw_data[1]['A']:
                print("controller disconnected")
                break

            # if raw_data[1]['RG']:
            #     gripper_closed = True
            # else:
            #     gripper_closed = False

            if raw_data[1]['RTr']:
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = 0.0

                pub.publish(msg)
                rate.sleep()
            else:

                relative_pos_t1 /= counter

                pos_diff = (relative_pos_t1 - relative_pos_t) / (time_stamp_t1 - time_stamp_t)
                pos_diff *= SCALE_VEL #3
                pos_diff_threshold = (np.abs(pos_diff) > MIN_DISCRETIZATION) * pos_diff
                relative_vel = np.clip(pos_diff_threshold, -MAX_ABS_VEL, MAX_ABS_VEL)


                # compute rotation different with logarithm
                # rot_diff = logm(np.matmul(relative_R_t1, relative_R_t.T)) / (time_stamp_t1 - time_stamp_t)
                # rot_diff = logm(np.matmul(relative_R_t, relative_R_t1.T)) / (time_stamp_t1 - time_stamp_t)
                # rot_diff = logm(np.matmul(relative_R_t.T, relative_R_t1)) / (time_stamp_t1 - time_stamp_t)
                rot_diff = logm(np.matmul(relative_R_t1.T, relative_R_t)) / (time_stamp_t1 - time_stamp_t)
                # angular_vel[0] = -rot_diff[0, 1]
                # angular_vel[1] = -rot_diff[1, 2]
                # angular_vel[2] = rot_diff[0, 2] #  #-rot_diff[0, 1]

                angular_vel[0] = - rot_diff[1, 2]
                angular_vel[1] = - rot_diff[0, 1]
                angular_vel[2] = rot_diff[0, 2]  # #-rot_diff[0, 1]


                angular_vel = (np.abs(angular_vel) > MIN_ROT_DISCRETIZATION) * angular_vel
                angular_vel *= SCALE_ROT
                angular_vel = np.clip(angular_vel, -MAX_ABS_ROT, MAX_ABS_ROT)

                pos_x = -relative_vel[2]
                pos_y = -relative_vel[0]
                pos_z = relative_vel[1]
                ori_x = 0# angular_vel[2]
                ori_y = 0# angular_vel[1]
                ori_z = 0# angular_vel[0]

                cur_data = np.array([pos_x, pos_y, pos_z, ori_x, ori_y, ori_z])
                data_buffer.append(cur_data)
                moving_avg = sum(data_buffer) / len(data_buffer)

                print("-------------------------------")
                print(moving_avg[:3])
                print(moving_avg[3:])

                msg.linear.x = moving_avg[0]
                msg.linear.y = moving_avg[1]
                msg.linear.z = moving_avg[2]
                msg.angular.x = moving_avg[3]
                msg.angular.y = moving_avg[4]
                msg.angular.z = moving_avg[5]

                # print("record action: " + str(relative_vel))
                # action = np.zeros(7)
                # action[0] = -relative_vel[2]
                # action[1] = -relative_vel[0]
                # action[2] = relative_vel[1]
                # action[3:6] = angular_vel
                # action[-1] = gripper_closed * 1.
                # print(action[:3])
                # env.step(action)
                pub.publish(msg)
                rate.sleep()

            relative_pos_t = relative_pos_t1.copy()
            relative_pos_t1 = np.zeros(3)
            relative_R_t = relative_R_t1.copy()
            time_stamp_t = time_stamp_t1
            counter = 0

    except KeyboardInterrupt:
        print("Shutting down")

    # env.release()


if __name__ == '__main__':
    main()
#
# data_folder = "/home/alfredo/PycharmProjects/robot_envs/files/data/yumi_drop/"
# list_folders = os.listdir(data_folder)
# img_tot = np.zeros((480, 640, 3))
# for folder in list_folders:
#     img_tot += imageio.imread(data_folder + folder + '/img0000.png')
# img_tot /= len(list_folders)
# img_diff = np.zeros((480, 640, 3))
# for folder in list_folders:
#     img_diff += np.abs(img_tot - imageio.imread(data_folder + folder + '/img0000.png'))
# img_diff /= len(list_folders)
# plt.imshow(img_diff.astype(int)*5)
# plt.show()