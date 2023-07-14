#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
import actionlib
import random
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from copy import deepcopy

import franka_gripper.msg
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Twist, WrenchStamped
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import *


from time import sleep


class pulling_franka:

    def __init__(self, namespace, robot):

        # Variables
        self.force_b = []
        self.torque_b = []
        self.force_s = []
        self.torque_s = []

        self.joint_pose = []

        self.threshold = False

        self.baer_init_pose = [0.5442778529651637, -0.3918180933114202, -0.644055285663591, -2.2240036563286165, 1.3288237952921125,
         1.606063438897899, 0.5011664036929228]
        self.stein_init_pose = [-0.2026565741948859, -0.3515336915241687, -0.3045408560727772, -2.180325759285375, -1.793592956690806,
         2.010203608140892, 1.0840929914664361]
        # self.victor = [-0.006448649147283548, -0.18572620676790708, 0.056528580207321995, -1.6648124255179138, 0.013624473211176689, 1.4953778528546542, 0.917335578738194]
        self.victor = [0.09301466155397815, -0.14200753322191403, -0.3639721526187258, -2.6205480761988516, -0.16606929467189527,
         2.4358248569700454, -0.859788213376758]
        # namespace can be either panda or nothing
        self.namespace = namespace
        self.robot = robot

        # if robot == 'baer':
        #     self.init_pose = self.baer_init_pose
        # else:
        self.init_pose = self.stein_init_pose
        self.init_pose = self.victor

        print(self.namespace + '/kth_joint_pose_effort_interface_controller/panda/equilibrium_pose')

        # Publish contents: cartesian velocity, joint pose
        self.joint_pose_pub = rospy.Publisher(self.namespace + '/kth_joint_pose_effort_interface_controller/panda/equilibrium_pose', PoseStamped)
        #self.joint_pose_pub = rospy.Publisher('/kth_joint_pose_effort_interface_controller/panda/equilibrium_pose', PoseStamped)

        self.cart_vel_pub = rospy.Publisher(self.namespace +'/kth_cartesian_velocity_effort_interface_controller/panda/equilibrium_pose', Twist)
        #self.cart_vel_pub = rospy.Publisher('/kth_cartesian_velocity_effort_interface_controller/panda/equilibrium_pose', Twist)

        #self.force_tresh_pub = rospy.Publisher('/status_forces', Bool, queue_size=10)

        # Subscribers: F/T sensors, images
        self.joint_pose_sub = rospy.Subscriber("/franka_state_controller/joint_states", JointState, self.callback_joints)

        self.force9_sub = rospy.Subscriber("/optoforce_node/wrench_UCE0B239", WrenchStamped, self.callback_baer)
        self.force2_sub = rospy.Subscriber("/optoforce_node/wrench_UCE0B232", WrenchStamped, self.callback_stein)

        self.check_forces_sub = rospy.Subscriber("/status_force", Bool, self.callback_stop)

        #while len(self.force_b) != 3 and not rospy.is_shutdown():
            #rospy.loginfo("F/T sensor not loaded yet")


        # client for gripper
        self.gripper_client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)
        self.gripper_client.wait_for_server()
        self.gripper_client_force = actionlib.SimpleActionClient('/franka_gripper/grasp',
                                                                 franka_gripper.msg.GraspAction)
        self.gripper_client_force.wait_for_server()
        self.gripper_client_stop = actionlib.SimpleActionClient('/franka_gripper/stop', franka_gripper.msg.StopAction)
        self.gripper_client_stop.wait_for_server()
        print("connected to grasping server")

        # controller services
        rospy.wait_for_service(self.namespace +"/controller_manager/load_controller")
        self.s_load_controller = rospy.ServiceProxy(self.namespace +'/controller_manager/load_controller', LoadController)


        # Load velocity controller
        name = "kth_cartesian_velocity_effort_interface_controller"
        resp = self.s_load_controller.call(LoadControllerRequest(name))

        if resp.ok:
            print("Loaded", name)
        else:
            print("Error when loading", name)

        # SWITCH
        rospy.wait_for_service(self.namespace +'/controller_manager/switch_controller')
        self.s_switch_controller = rospy.ServiceProxy(self.namespace +'/controller_manager/switch_controller', SwitchController)

    # define callbacks for forces!!
    def callback_baer(self, msg):
        temp_force = []
        temp_torque = []
        temp_torque.append(msg.wrench.torque.x)
        temp_force.append(msg.wrench.force.x)
        temp_torque.append(msg.wrench.torque.y)
        temp_force.append(msg.wrench.force.y)
        temp_torque.append(msg.wrench.torque.z)
        temp_force.append(msg.wrench.force.z)
        # rospy.loginfo("Inside baer\n")

        self.force_b = deepcopy(temp_force)
        self.torque_b = deepcopy(temp_torque)

        # Check threshold
        if np.linalg.norm(self.force_b) > 20:
            self.threshold = True

        #self.force_tresh_pub.publish(self.threshold)

        # rospy.loginfo("force baer: \n{}\n".format(force_b))
        # rospy.loginfo("force threshold: \n{}\n".format(self.threshold))

    def callback_stein(self, msg):
        temp_force = []
        temp_torque = []
        temp_torque.append(msg.wrench.torque.x)
        temp_force.append(msg.wrench.force.x)
        temp_torque.append(msg.wrench.torque.y)
        temp_force.append(msg.wrench.force.y)
        temp_torque.append(msg.wrench.torque.z)
        temp_force.append(msg.wrench.force.z)
        # rospy.loginfo("Inside baer\n")

        self.force_s = deepcopy(temp_force)
        self.torque_s = deepcopy(temp_torque)

        # Check threshold
        if np.linalg.norm(self.force_s) > 20:
            self.threshold = True

        #self.force_tresh_pub.publish(self.threshold)

        # rospy.loginfo("force baer: \n{}\n".format(force_b))
        # rospy.loginfo("force threshold: \n{}\n".format(self.threshold))

    def callback_stop(self, msg):
        print(msg.data)
        if msg.data == True:
            joint_controller = ['kth_joint_pose_effort_interface_controller']
            velocity_controller = ['kth_cartesian_velocity_effort_interface_controller']
            self.swap_controller(joint_controller, velocity_controller)

    def callback_joints(self, msg):
        self.joint_pose = msg.position
        # print(self.joint_pose)


    def swap_controller(self, name_start, name_stop):
        # do the swithcing
        resp = self.s_switch_controller.call(name_start, name_stop, 1, False, 0.0)
        print(resp)
        if resp.ok == 1:
            print("Started " + name_start + " successfully")
            print("Stopped " + name_stop + " successfully")
        else:
            print("Error when  " + name_start + " starting")
            print("Error when  " + name_stop + " stopping")

    def send_initial_pose(self,):
        global goal
        goal = PoseStamped()

        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"  # TODO: check this frame ID

        initial = self.baer_init_pose
        initial = self.victor
        # initial = self.stein_init_pose
        initial = [0.09301466155397815, -0.14200753322191403, -0.3639721526187258, -2.6205480761988516, -0.16606929467189527, 2.4358248569700454, -0.859788213376758]

        #if self.robot == 'stein':
       #     initial = self.stein_init_pose

        goal.pose.position.x = initial[0]
        goal.pose.position.y = initial[1]
        goal.pose.position.z = initial[2]

        goal.pose.orientation.x = initial[3]
        goal.pose.orientation.y = initial[4]
        goal.pose.orientation.z = initial[5]
        goal.pose.orientation.w = initial[6]

        self.joint_pose_pub.publish(goal)



    def gripper_grasp_no_init(self, width=0.00018, e_inner=0.001, e_outer=0.001, speed=0.05, force=20):

        self.gripper_client.send_goal(
            franka_gripper.msg.GraspGoal(
                width,
                franka_gripper.msg.GraspEpsilon(e_inner, e_outer),
                speed,
                force
            )
        )
        return self.gripper_client.wait_for_result()

    def command_velocity(self):
        # print(self.threshold)
        global msg
        msg = Twist()
        # if self.threshold == False:
        msg.linear.x = 0.0
        msg.linear.y = -0.02
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        # else:
        #     msg.linear.x = 0.0
        #     msg.linear.y = 0.0
        #     msg.linear.z = 0.0
        #     msg.angular.x = 0.0
        #     msg.angular.y = 0.0
        #     msg.angular.z = 0.0

        rate = rospy.Rate(10)
        for i in range(3):
            info = "Commandig velocity: " + str(msg.linear.y)
            rospy.loginfo(info)
            self.cart_vel_pub.publish(msg, latch=True)
            rate.sleep()

def print_commands():
    print("-------------------------------------")
    print("init - go to start pose")
    print("go - start moving robot")
    print("og - open gripper")
    print("cg - close gripper")
    print("s - stop robot")
    print("vc - velocity control")
    print("pc - position control")
    print("com - plot commands")
    print("--------------------------------------")

def main(args):
    rospy.init_node('pulling', anonymous=False)
    print("starting")

    panda_robot = 'baer'
    panda_ns = ''

    panda = pulling_franka(panda_ns, panda_robot)
    print_commands()
    readinput = True
    rate = rospy.Rate(1000)

    joint_controller = ['kth_joint_pose_effort_interface_controller']
    velocity_controller = ['kth_cartesian_velocity_effort_interface_controller']

    try:
        while not (rospy.is_shutdown()):
            # check input
            if readinput:
                r_str = raw_input()
            if (r_str == '\x03'):
                break

            if r_str == 'init':
                print('*** sending to initial position ***')
                try:
                    switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
                    ret = switch_controller(joint_controller, velocity_controller, 2, False, 0.0)
                    panda.send_initial_pose()
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)

                # readinput = False
                r_str = 'wait'

            if r_str == 'go':
                print('*** Commanding velocity ***')
                try:
                    switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
                    ret = switch_controller(velocity_controller, joint_controller, 2, False, 0.0)
                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)

                panda.command_velocity()
                # r_str = 'wait'

            if r_str == 'vc':
                print('*** To velocity control ***')
                try:
                    switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
                    ret = switch_controller(velocity_controller, joint_controller, 2, False, 0.0)
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)

            if r_str == 'pc':
                print('*** To position control ***')
                try:
                    switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
                    ret = switch_controller(joint_controller, velocity_controller, 2, False, 0.0)
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)

            if r_str == 'cg':
                print("*** Closing gripper ***")
                panda.gripper_grasp_no_init(width=0.0001)

            if r_str == 'og':
                print("*** Opening gripper ***")
                panda.gripper_grasp_no_init(width=0.04)


            if r_str == 's':
                print("*** Stopping robot gripper ***")
                switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
                ret = switch_controller(joint_controller, velocity_controller, 2, False, 0.0)

            if r_str == 'com':
                print_commands()

            if r_str == 'wait':
                rate.sleep()

            rate.sleep()

    except KeyboardInterrupt:
        print("Shutting down")







  # print('*** Commanding velocity ***')
  # try:
  #   panda.command_velocity()
  # except:
  #     print('Commanding velocity failed')



  #rospy.spin()


if __name__ == '__main__':
    main(sys.argv)