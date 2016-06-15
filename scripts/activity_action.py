#! /usr/bin/env python
__author__ = 'm_alomari'
import roslib
import sys, os
import rospy
import yaml
import actionlib
import rosbag
import getpass, datetime
import shutil
from std_msgs.msg import String
from scitos_ptu.msg import *
from skeleton_manager import SkeletonManager
from online_activity_recognition.msg import recogniseAction, recogniseActionResult, skeleton_message
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion
import math

class activity_server(object):
    def __init__(self):
        # Start server
        rospy.loginfo("Activity Recognition starting an action server")
        self.skeleton_msg = None
        self._as = actionlib.SimpleActionServer("recognise_action", recogniseAction, \
                                                    execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.sk_publisher = SkeletonManager()
        self.skeleton_msg = skeleton_message()  #default empty
        self.filepath = os.path.join(roslib.packages.get_pkg_dir("online_activity_recognition"), "config")
        try:
            self.config = yaml.load(open(os.path.join(self.filepath, 'config.ini'), 'r'))
            print "config loaded.."
        except:
            print "no config file found"

        # PTU state - based upon current_node callback
        self.ptu_action_client = actionlib.SimpleActionClient('/SetPTUState', PtuGotoAction)
        self.ptu_action_client.wait_for_server()


        rospy.Subscriber("/robot_pose", Pose, callback=self.robot_callback, queue_size=10)

        #request_sent
        self.request_sent_flag = 0


    def execute_cb(self, goal):
        duration = goal.duration
        start = rospy.Time.now()
        end = rospy.Time.now()
        self.sk_publisher._initialise_data()
        self.sk_publisher.robot_pose_flag = 1

        # print goal
        self.set_ptu_state(goal.waypoint)
        self.get_rotation_matrix(goal.waypoint)

        prev_uuid = ""

        while (end - start).secs < duration.secs:
            if self._as.is_preempt_requested():
                break

            self.sk_publisher.get_skeleton()
            print self.sk_publisher.accumulate_data.keys()
            # print self.pan,self.tilt
            print self.rot,self.pos_robot
            # print self.robot_pose.orientation
            print '------------'

            rospy.sleep(0.01)  # wait until something is published

            end = rospy.Time.now()

        # after the action reset everything
        self.reset_all()

        self._as.set_succeeded(recogniseActionResult())


    def get_rotation_matrix(self, waypoint):
        # print waypoint

        try:
            pan = self.config[waypoint]['pan']
            tilt = self.config[waypoint]['tilt']
        except KeyError:
            print 'could not find the waypoint, ptu set to 0,0'

        xr = self.robot_pose.position.x
        yr = self.robot_pose.position.y
        zr = self.robot_pose.position.z

        ax = self.robot_pose.orientation.x
        ay = self.robot_pose.orientation.y
        az = self.robot_pose.orientation.z
        aw = self.robot_pose.orientation.w
        roll, pitch, yaw = euler_from_quaternion([ax, ay, az, aw])    #odom
        yaw   += pan*math.pi / 180.                   # this adds the pan of the ptu state when recording took place.
        pitch += tilt*math.pi / 180.                # this adds the tilt of the ptu state when recording took place.
        rot_y = np.matrix([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        rot_z = np.matrix([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        self.rot = rot_z*rot_y
        self.pos_robot = np.matrix([[xr], [yr], [zr+1.66]]) # robot's position in map frame

    def robot_callback(self, msg):
        self.robot_pose = msg


    def reset_all(self):
        # self.image_logger.stop_image_callbacks = 0   #stop the image callbacks in the logger
        # self.sk_publisher.robot_pose_flag = 0        #stop the callbacks in the pub
        self.reset_ptu()


    def reset_ptu(self):
        ptu_goal = PtuGotoGoal();
        ptu_goal.pan = 0
        ptu_goal.tilt = 0
        ptu_goal.pan_vel = 30
        ptu_goal.tilt_vel = 30
        self.ptu_action_client.send_goal(ptu_goal)
        self.ptu_action_client.wait_for_result()


    def set_ptu_state(self, waypoint):
        ptu_goal = PtuGotoGoal();
        try:
            ptu_goal.pan = self.config[waypoint]['pan']
            ptu_goal.tilt = self.config[waypoint]['tilt']
            ptu_goal.pan_vel = self.config[waypoint]['pvel']
            ptu_goal.tilt_vel = self.config[waypoint]['tvel']
            self.ptu_action_client.send_goal(ptu_goal)
            self.ptu_action_client.wait_for_result()
        except KeyError:
            self.reset_ptu()


if __name__ == "__main__":
    rospy.init_node('activity_action_server')

    activity_server()
    rospy.spin()
