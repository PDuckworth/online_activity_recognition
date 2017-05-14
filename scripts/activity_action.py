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
import numpy as np
from qsrlib_io.world_trace import Object_State, World_Trace

from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_qsr_trace import World_QSR_Trace
from qsrlib_utils.utils import merge_world_qsr_traces
from qsrlib_qstag.qstag import Activity_Graph
from qsrlib_qstag.utils import *
import copy
import cv2
import colorsys
import operator

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cPickle as pickle
import time

class activity_server(object):
    def __init__(self):
        # subscribe to robot pose to get location
        rospy.Subscriber("/robot_pose", Pose, callback=self.robot_callback, queue_size=10)

        # # get some objects
        # datafilepath = os.path.join(roslib.packages.get_pkg_dir("online_activity_recognition"), "data")
        datafilepath = '/home/' + getpass.getuser() + '/SkeletonDataset/Learning/accumulate_data/'

        # self.objects = self.get_soma_objects()
        # path = os.path.join(datafilepath, 'point_cloud_object_clusters')
        # self.objects = self.get_point_cloud_objects(path)

        self.soma_id_store = MessageStoreProxy(database='message_store', collection='soma_activity_ids_list')
        self.soma_store = MessageStoreProxy(database="somadata", collection="object")
        self.get_soma_objects()



        # Start server
        rospy.loginfo("Activity Recognition starting an action server")
        self.skeleton_msg = None
        self._as = actionlib.SimpleActionServer("recognise_action", recogniseAction, \
                                                    execute_cb=self.execute_cb, auto_start=False)

        self.sk_publisher = SkeletonManager()   # Import new skel logger from activity_data?
        # self.skeleton_msg = skeleton_message()  #default empty

        self.filepath = os.path.join(roslib.packages.get_pkg_dir("online_activity_recognition"), "config")
        try:
            self.config = yaml.load(open(os.path.join(self.filepath, 'config.ini'), 'r'))
            print "config loaded.."
        except:
            print "no config file found"

        # PTU state - based upon current_node callback
        self.ptu_action_client = actionlib.SimpleActionClient('/SetPTUState', PtuGotoAction)
        print "pan tilt unit..."
        self.ptu_action_client.wait_for_server()

        # load files
        self.load_all_files(datafilepath)
        # online window of QSTAGS
        self.windows_size = 100
        self.th = 4         # column thickness
        self.th2 = 4        # frame thickness
        self.th3 = 4        # thickness between images
        self.th_100 = 200   # how big is the 100%
        self.qsr_median_window = 1   # smooth the QSR relations
        self.online_window = {}
        self.online_window_img = {}
        self.act_results = {}

        self.image_pub = rospy.Publisher("/activity_recognition_results", Image, queue_size=10)
        self.image_label = cv2.imread(datafilepath+'/image_label.png')
        self.bridge = CvBridge()
        self._as.start()

        # self.soma_roi_config = {
        #                 'KitchenTableLow':'Kitchen', 'KitchenTableHigh':'Kitchen',
        #                 'KitchenCounter1':'Kitchen', 'KitchenDemo':'Kitchen','KitchenCounter2':'Kitchen',
        #                 'KitchenCounter3':'Kitchen', 'ReceptionDesk':'Reception', 'HospActRec1':'Hospitality',
        #                 'HospActRec4':'Hospitality', 'CorpActRec3':'Corporate', 'SuppActRec1': 'Support' }


    def execute_cb(self, goal):
        self.online_window = {}
        self.online_window_img = {}
        self.act_results = {}
        duration = goal.duration
        start = rospy.Time.now()
        end = rospy.Time.now()

        # self.sk_publisher._initialise_data()
        # self.sk_publisher.robot_pose_flag = 1
        # self.waypoint = goal.waypoint

        self.set_ptu_state(goal.waypoint)  # set this by selecting an object in the roi ?
        # self.get_rotation_matrix()
        # prev_uuid = ""
        if goal.waypoint == "offline":
            print "running offline..."

            self.sk_publisher.offline = 1
            video = "vid50"
            self.sk_publisher.offline_directory = "/home/scpd/Datasets/ECAI_Data/dataset_segmented_15_12_16/"
            self.sk_publisher.run_offline_instead_of_callback(video)
        else:
            self.sk_publisher.action_called = 1

        while (end - start).secs < duration.secs:
            if self._as.is_preempt_requested():
                print ">> preempt_requested\n"
                break

            self.convert_to_map()

            # THIS SHOULD USE MONGO like the logger does
            # region = self.soma_roi_config[self.waypoint]
            self.get_world_frame_trace()

            self.update_online_window()
            self.recognise_activities()
            self.plot_online_window()
            # print '------------'
            rospy.sleep(0.01)  # wait until something is published

            end = rospy.Time.now()
        # after the action reset everything
        self.reset_all()
        self._as.set_succeeded(recogniseActionResult())
        print ">> set_succeeded\n"


    def get_soma_objects(self):
        """srv call to mongo and get the list of new objects and locations"""

        ids = self.soma_id_store.query(String._type)
        ids = [id_[0].data for id_ in ids]
        print "SOMA IDs used to observe >> ", ids

        objs = self.soma_store.query(SOMAObject._type, message_query = {"id":{"$in": ids}})
        all_objects = {}
        self.soma_objects = {}
        for (ob, meta) in objs:
            if ob.id in ids:
                k = ob.type + "_" + ob.id
                p = ob.pose.position
                all_objects[k] = (p.x, p.y, p.z)
        #print "\nall_objects> ", all_objects.keys()

        for r, poly in self.rois.items():
            self.soma_objects[r] = {}
            for (ob, (x,y,z)) in all_objects.items():
                if poly.contains(Point([x, y])):
                    self.soma_objects[r][ob] = (x,y,z)
        print "objects >> ", [self.soma_objects[k].keys() for k in self.soma_objects.keys()]

    def plot_online_window(self):
        if len(self.online_window_img) == 0:
            img = np.zeros((len(self.code_book)*self.th+self.th3+self.th_100,  self.windows_size*self.th2+59,  3),dtype=np.uint8)+255
        else:
            img = np.zeros((len(self.code_book)*self.th+self.th3+self.th_100,  self.windows_size*self.th2*len(self.online_window_img)+59,  3),dtype=np.uint8)+255
        img[len(self.code_book)*self.th:len(self.code_book)*self.th+self.th3,:,:] = 120
        h = len(self.code_book)*self.th+self.th3+self.th_100
        img[h-356:h,0:59,:] = self.image_label
        img[:,50:59,:] = 200

        img2 = np.zeros((self.th_100,self.windows_size*self.th2,3),dtype=np.uint8)+255
        for counter,subj in enumerate(self.online_window_img):
            # print 'test',counter,subj
            img1 = self.online_window_img[subj]
            #cv2.imshow('test'+str(counter),self.online_window_img[subj])
            img2 = np.zeros((self.th_100,self.windows_size*self.th2,3),dtype=np.uint8)+255
            for f in range(self.windows_size):
                to_be_ordered = {}
                for act in self.act_results[subj]:
                    to_be_ordered[act] = self.act_results[subj][act][f]
                sorted_x = sorted(to_be_ordered.items(), key=operator.itemgetter(1))
                for x in reversed(sorted_x):
                    img2[int(self.th_100-x[1]*self.th_100/100.0):self.th_100,f*self.th2:(f+1)*self.th2,:] = self.RGB_tuples[x[0]]

            img[0:len(self.code_book)*self.th,  59+counter*self.windows_size*self.th2:59+(counter+1)*self.windows_size*self.th2,  :] = img1
            img[len(self.code_book)*self.th+self.th3:,  59+counter*self.windows_size*self.th2:59+(counter+1)*self.windows_size*self.th2,  :] = img2
            img[:,  58+(counter+1)*self.windows_size*self.th2:59+(counter+1)*self.windows_size*self.th2,  :] = 120
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img2, "bgr8"))
            # self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)

        # cv2.imwrite('/home/omari/test.png',img)
        #     cv2.imshow('actions',img)
        cv2.waitKey(1)

    def recognise_activities(self):
        self.act_results = {}
        for subj, window in self.online_window.items():
            # compressing the different windows to be processed
            for w in range(2,10,2):
                for i in range(self.windows_size-w):
                    compressed_window = copy.deepcopy(window[i,:])
                    for j in range(1,w+1):
                        compressed_window += window[j+i,:]

                    compressed_window = [1 if o !=0 else 0 for o in compressed_window]
                    # compressed_window /= compressed_window

                    # comparing the processed windows with the different actions
                    if subj not in self.act_results:
                        self.act_results[subj] = {}
                    for act in self.actions_vectors:
                        if act not in self.act_results[subj]:
                            self.act_results[subj][act] = np.zeros((self.windows_size), dtype=np.float32)

                        result = np.sum(compressed_window*self.actions_vectors[act])
                        if result != 0:
                            self.act_results[subj][act][i:i+w] += result
                        # if act==2:
                        #     self.act_results[subj][act][i:i+w] += 20
        # calibration
        for subj in self.act_results:
            for act in self.act_results[subj]:
                self.act_results[subj][act] /= 20

    def update_online_window(self):
        for subj in self.subj_world_trace:
            #print subj
            # initiate the window of QSTAGS for this person
            if subj not in self.online_window:
                self.online_window[subj] = np.zeros((self.windows_size, len(self.code_book)), dtype=np.uint8)
                self.online_window_img[subj] = np.zeros((len(self.code_book)*self.th,self.windows_size*self.th2,3),dtype=np.uint8)+255
            # shift one frame
            else:
                self.online_window[subj][1:self.windows_size] = self.online_window[subj][0:self.windows_size-1]
                self.online_window_img[subj][:,self.th2:self.windows_size*self.th2,:] = self.online_window_img[subj][:,0:self.windows_size*self.th2-self.th2,:]
            # find which QSTAGS happened in this frame
            ret = self.subj_world_trace[subj]
            self.online_window[subj][0,:] = 0
            self.online_window_img[subj][:, 0:self.th2, :] = 255
            #print self.code_book
            #print ret.qstag.graphlets.graphlets

            for cnt, h in zip(ret.qstag.graphlets.histogram, ret.qstag.graphlets.code_book):
                # print ">>>", cnt, h #, ret.qstag.graphlets.graphlets[h]

                if isinstance(h, int):
                    h = "{:20d}".format(h).lstrip()
                # print h, len(self.code_book), type(self.code_book), self.code_book.shape
                code_book = [str(i) for i in self.code_book]
                if str(h) in code_book:  # Futures WARNING here
                    index = list(code_book).index(h)
                    # print ">>>HERE"
                    self.online_window[subj][0,index] = 1
                    self.online_window_img[subj][index*self.th:index*self.th+self.th, 0:self.th2, :] = 10


    def load_all_files(self, path):
        print "loading files..."
        # date = time.strftime("%d_%m_%Y")
        # date = '15_06_2016'  ## test features :)
        #
        # with open(path + "/code_book_" + date + ".p", 'r') as f:
        #     self.code_book = pickle.load(f)
        #
        # with open(path + "/graphlets_" + date + ".p", 'r') as f:
        #     self.graphlets = pickle.load(f)
        #
        # self.actions_vectors = {}
        # with open(path + "/v_singular_mat_" + date + ".p", 'r') as f:
        #     VT = pickle.load(f)

        # with open(path + "/code_book_MK3.p", 'r') as f:
        #     self.code_book = pickle.load(f)
        # print "codebook:", len(self.code_book)

        date_files = sorted([f for f in listdir(path)])
        print "??", date_files
        path = os.path.join(path, date_files[-1])

        with open(path + "/code_book.p", 'r') as f:
            self.code_book = pickle.load(f)
        with open(path + "/graphlets.p", 'r') as f:
            graphlets = pickle.load(f)


        # self.code_book = data[0]
        print "codebook:", len(self.code_book)
        # graphlets = data[1]
        # for cnt, g in enumerate(graphlets):
        #     os, ss, ts = nodes(g)
        #     ssl = [d.values() for d in ss]

        # with open(path + "/graphlets_MK3.p", 'r') as f:
        #     self.graphlets = pickle.load(f)

        self.actions_vectors = {}
        with open(path + "/topic_word.p", 'r') as f:
            VT = pickle.load(f)
            print "actions:", VT.shape

        N = 0
        for count,act in enumerate(VT):

            p_sum = sum(x for x in act if x > 0)    # sum of positive graphlets
            self.actions_vectors[count] = act/p_sum*100
            self.actions_vectors[count][self.actions_vectors[count]<0] = 0
            #print self.actions_vectors[count]
            #print '----'
            N+=1
        HSV_tuples = [(x*1.0/N, 0.7, 0.9) for x in range(N)]
        self.RGB_tuples = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples)
        for c,i in enumerate(self.RGB_tuples):
            self.RGB_tuples[c] = [255*x for x in i]
            # print self.RGB_tuples[c]

	#self.RGB_tuples[0] = [255, 0 ,0]
	#self.RGB_tuples[1] = [0, 255 ,0]
	#self.RGB_tuples[2] = [0, 0 ,255]
	#self.RGB_tuples[3] = [255, 255 ,0]
	#self.RGB_tuples[4] = [0, 255 ,255]
	#self.RGB_tuples[5] = [255, 0 ,255]
	#self.RGB_tuples[6] = [20, 20 ,20]
	#self.RGB_tuples[7] = [200, 200 ,200]

    def get_object_frame_qsrs(self, world_trace):
        joint_types = {'left_hand': 'hand', 'right_hand': 'hand',  'head-torso': 'tpcc-plane'}

        joint_types_plus_objects = joint_types.copy()
        for object in self.objects:
            generic_object = "_".join(object.split("_")[:-1])
            joint_types_plus_objects[object] = generic_object
        #print joint_types_plus_objects

        """create QSRs between the person's joints and the soma objects in map frame"""
        qsrs_for=[]
        for ob in self.objects:
            qsrs_for.append((str(ob), 'left_hand'))
            qsrs_for.append((str(ob), 'right_hand'))
            #qsrs_for.append((str(ob), 'torso'))

        dynamic_args = {}
        # dynamic_args['argd'] = {"qsrs_for": qsrs_for, "qsr_relations_and_values": {'Touch': 0.25, 'Near': 0.5,  'Ignore': 10}}
        # dynamic_args['qtcbs'] = {"qsrs_for": qsrs_for, "quantisation_factor": 0.05, "validate": False, "no_collapse": True} # Quant factor is effected by filters to frame rate
        # dynamic_args["qstag"] = {"object_types": joint_types_plus_objects, "params": {"min_rows": 1, "max_rows": 1, "max_eps": 2}}

        dynamic_args['argd'] = {"qsrs_for": qsrs_for, "qsr_relations_and_values": {'Touch': 0.5, 'Near': 0.1, 'Away': 1.5, 'Ignore': 10}}
        # dynamic_args['argd'] = {"qsrs_for": qsrs_for, "qsr_relations_and_values": {'Touch': 0.25, 'Near': 1.0, 'Away': 2.0, 'Ignore': 10}}
        dynamic_args['qtcbs'] = {"qsrs_for": qsrs_for, "quantisation_factor": 0.01, "validate": False, "no_collapse": True} # Quant factor is effected by filters to frame rate
        dynamic_args["qstag"] = {"object_types": joint_types_plus_objects, "params": {"min_rows": 1, "max_rows": 1, "max_eps": 3}}
        dynamic_args["filters"] = {"median_filter": {"window": self.qsr_median_window}}

        qsrlib = QSRlib()
        req = QSRlib_Request_Message(which_qsr=["argd", "qtcbs"], input_data=world_trace, dynamic_args=dynamic_args)
        req = QSRlib_Request_Message(which_qsr="argd", input_data=world_trace, dynamic_args=dynamic_args)
        ret = qsrlib.request_qsrs(req_msg=req)

        print "\n"
        for ep in ret.qstag.episodes:
            print ep
        return ret


    def get_world_frame_trace(self):
        """Accepts a dictionary of world (soma) objects.
        Adds the position of the object at each timepoint into the World Trace"""
        self.subj_world_trace = {}


        for subj in self.skeleton_map:
            print ">", len(self.skeleton_map[subj]["left_hand"])
            ob_states={}
            world = World_Trace()
            map_frame_data = self.skeleton_map[subj]
            for joint_id in map_frame_data.keys():

                #Joints:
                for t in xrange(self.frames):
                    x = map_frame_data[joint_id][t][0]
                    y = map_frame_data[joint_id][t][1]
                    z = map_frame_data[joint_id][t][2]
                    if joint_id not in ob_states.keys():
                        ob_states[joint_id] = [Object_State(name=joint_id, timestamp=t+1, x=x, y=y, z=z)]
                    else:
                        ob_states[joint_id].append(Object_State(name=joint_id, timestamp=t+1, x=x, y=y, z=z))

            # SOMA objects
            for t in xrange(self.frames):
                for object, (x,y,z) in self.objects.items():
                    if object not in ob_states.keys():
                        ob_states[object] = [Object_State(name=str(object), timestamp=t+1, x=x, y=y, z=z)]
                    else:
                        ob_states[object].append(Object_State(name=str(object), timestamp=t+1, x=x, y=y, z=z))

                # # Robot's position
                # (x,y,z) = self.robot_data[t][0]
                # if 'robot' not in ob_states.keys():
                #     ob_states['robot'] = [Object_State(name='robot', timestamp=t, x=x, y=y, z=z)]
                # else:
                #     ob_states['robot'].append(Object_State(name='robot', timestamp=t, x=x, y=y, z=z))

            for obj, object_state in ob_states.items():
                world.add_object_state_series(object_state)

            # get world trace for each person
            # region = self.soma_roi_config[self.waypoint]
            self.subj_world_trace[subj] = self.get_object_frame_qsrs(world)

            if self.sk_publisher.offline == 1:
                self.skeleton_map[subj]["left_hand"] = self.skeleton_map[subj]["left_hand"][2:]
                self.skeleton_map[subj]["right_hand"] = self.skeleton_map[subj]["right_hand"][2:]



    def convert_to_map(self):
        self.skeleton_map = {}
        frames = 20      # frames to be processed
        self.frames = frames

        for subj in self.sk_publisher.accumulate_data.keys():

            all_data = len(self.sk_publisher.accumulate_data[subj])
            if all_data<frames*2:
                continue
            print all_data
            if self.sk_publisher.offline == 1:
                new_range = range(0,frames*2, 2)
            else:
                new_range = range(np.max([0, all_data-frames*2]), all_data,2)

            self.skeleton_map[subj] = {}
            self.skeleton_map[subj]['right_hand'] = []
            self.skeleton_map[subj]['left_hand'] = []
            for f in new_range:
                print '*',f
                robot_pose = self.sk_publisher.accumulate_robot[subj][f]

                for j, name in zip([7, 3],["right_hand", "left_hand"]):
                    hand = self.sk_publisher.accumulate_data[subj][f].joints[j].pose.position
                    map_point = self.sk_publisher.convert_to_world_frame(hand, robot_pose)
                    map_joint = [map_point.x, map_point.y, map_point.z]
                    self.skeleton_map[subj][name].append(map_joint)
                    # import pdb; pdb.set_trace()

            if self.sk_publisher.offline == 1:
                self.sk_publisher.accumulate_data[subj] = self.sk_publisher.accumulate_data[subj][2:]


                # map_joint = self._convert_one_joint_to_map(right_hand)
                # map_joint = self._convert_one_joint_to_map(left_hand)
                # self.skeleton_map[subj]['left_hand'].append(map_joint)
        # print self.skeleton_map[subj]

    # def _convert_one_joint_to_map(self,joint):
    #     y = joint.pose.position.x
    #     z = joint.pose.position.y
    #     x = joint.pose.position.z
    #     pos_p = np.matrix([[x], [-y], [z]])     # person's position in camera frame
    #     map_pos = self.rot*pos_p+self.pos_robot # person's position in map frame
    #     x_mf = map_pos[0,0]
    #     y_mf = map_pos[1,0]
    #     z_mf = map_pos[2,0]
    #     return [x_mf,y_mf,z_mf]


    # def get_rotation_matrix(self):
    #
    #     try:
    #         pan = float(self.sk_publisher.ptu_pan)
    #         tilt = float(self.sk_publisher.ptu_tilt)
    #         # pan = self.config[waypoint]['pan']
    #         # tilt = self.config[waypoint]['tilt']
    #     except KeyError:
    #         print 'could not listen to ptu settings'
    #         pan, tilt = float(0), float(0)
    #
    #     xr = self.robot_pose.position.x
    #     yr = self.robot_pose.position.y
    #     zr = self.robot_pose.position.z
    #
    #     ax = self.robot_pose.orientation.x
    #     ay = self.robot_pose.orientation.y
    #     az = self.robot_pose.orientation.z
    #     aw = self.robot_pose.orientation.w
    #
    #     roll, pitch, yaw = euler_from_quaternion([ax, ay, az, aw])    #odom
    #     yaw   += pan*math.pi / 180.                   # this adds the pan of the ptu state when recording took place.
    #     pitch += tilt*math.pi / 180.                # this adds the tilt of the ptu state when recording took place.
    #     rot_y = np.matrix([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    #     rot_z = np.matrix([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    #     self.rot = rot_z*rot_y
    #     self.pos_robot = np.matrix([[xr], [yr], [zr+1.66]]) # robot's position in map frame


    def robot_callback(self, msg):
        self.robot_pose = msg

    def reset_all(self):
        # self.image_logger.stop_image_callbacks = 0   #stop the image callbacks in the logger
        # self.sk_publisher.robot_pose_flag = 0        #stop the callbacks in the pub
        self.sk_publisher.action_called = 0
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

    def get_fixed_objects(region=None):
        objects = {}
        objects['Kitchen'] = {}
        objects['Reception'] = {}
        objects['Hopotality'] = {}
        objects['Corporate'] = {}
        objects['Support'] = {}

        objects['Kitchen'] = {
        #'Microwave_1':  (-53.894511011092348, -5.6271549435167918, 1.2075203138621333),
        #'Microwave_2':  (-52.294511011092348, -5.6271549435167918, 1.2075203138621333),
        'Microwave_1':  (-52.294511011092348, -5.6271549435167918, 1.2075203138621333),
        'Sink_2':  (-55.902430164089097, -5.3220418631789883, 0.95348616325025226),
        'Fruit_bowl_3':  (-55.081272358597374, -8.5550720977828973, 1.2597648941515749),
        #'Fruit_bowl_11':  (-8.957, -17.511, 1.1),
        'Dishwasher_4':  (-55.313495480985964, -5.822285141172836, 0.87860846828010275),
        'Coffee_Machine_5': (-50.017233832554183, -5.4918825204775921, 1.3139597647929069)
        }
        objects['Reception'] = {
        'Coffee_Machine_6': (-5.5159040452346737, 28.564135219405774, 1.3149322505645362),
        #'Fridge_7': (-50.35, -5.24, 1.51)
        }
        objects['Hospitality'] = {
        'Printer_8':  (-1.6876578896088092, -5.9433505603441326, 1.1084470787101761),
        'Sink_9':  (2.9, -3, 1.1),
        #'Coffee_Machine_10': (-50.35, -5.24, 1.51)
        }
        objects['Corporate'] = {
        'Printer_11':  (-23.734682053245283, -14.096880839756942, 1.106873440473277),
        }
        objects['Support'] = {
        #'Printer_12':  (-8.957, -17.511, 1.1),
        }
        return objects

    def get_point_cloud_objects(self, path):
        objects = {}
        # allowed_objects = ['22','13','21','23','4','17','11','15','19','30','28','3','44','0','12']
        # Objects clusters extracted from point clouds by Nils, then filtered based on a where the human hands interact with (applying a 3% margin around the BBs).
        allowed_objects = ['21','13','17','19','4','22','18','15','23','3','11','24','16','12']
        for file in sorted(os.listdir(path)):
            if file.endswith(".txt"):
                file_num = file.replace("cluster_","").replace(".txt","")
                if file_num not in allowed_objects: continue
                with open(os.path.join(path,file),'r') as f:
                    f.readline()
                    line=f.readline()
                    (labs,xyz) = line.replace("\n","").split(":")
                    x,y,z = xyz.split(",")
                    string = "object_%s_%s" % (int(file_num),int(file_num))
                    objects[string] = (float(x),float(y),float(z)) # hack to keep file_num when object type is passed to QSRLib

        # objects["object_21_21"] = (-7.7,-35.0,1.0)
        # objects["object_13_13"] = (-7.0,-34.2,1.0)
        # objects["object_17_17"] = (-8.5,-31.0,1.0)
        return objects

def nodes(graph):
    """Getter.
    """
    # Get object nodes from graph
    objects, spa, temp = [], [], []
    for node in graph.vs():
        if node['node_type'] == 'object':
            objects.append(node['name'])
        if node['node_type'] == 'spatial_relation':
            spa.append(node['name'])
        if node['node_type'] == 'temporal_relation':
            temp.append(node['name'])
    return objects, spa, temp

if __name__ == "__main__":
    rospy.init_node('activity_action_server')

    activity_server()
    rospy.spin()
