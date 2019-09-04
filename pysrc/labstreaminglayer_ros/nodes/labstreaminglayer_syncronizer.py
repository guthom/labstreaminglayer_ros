#!/usr/bin/env python
import sys, os

#find better way to do this.
projectDir = os.path.join(os.path.dirname(__file__), '../')
sys.path.append(projectDir + "../thirdparty/liblsl-Python")

import nodeconfig as config
import rospy
import threading

from data.mapper_LSLToROS import Mapper_LSLtoROS
from data.mapper_ROSToLSL import Mapper_ROStoLSL

class LabStreaminLayerSynchronizer:


    def __init__(self):
        self.mapper = []

        self.cyclicMode = True
        self.rawTopics_LSLtoROS = None
        self.rawTopics_ROStoLSL = None
        self.nodeTopic = ""

        self.useLSLTypesBidirectional = True
        self.includeLSLTimestamps = True

    def Init(self):
        rospy.init_node(config.nodeName)
        self.nodeTopic = rospy.get_namespace() + rospy.get_name()
        self.InitParameter()
        self.InitLSLtoROS()
        self.InitROStoLSL()
        pass

    def InitParameter(self):
        self.cyclicMode = rospy.get_param(self.nodeTopic + "/cyclicMode", default=True)
        self.useLSLTypesBidirectional = rospy.get_param(self.nodeTopic + "/useLSLTypesBidirectional", default=False)
        self.includeLSLTimestamps = rospy.get_param(self.nodeTopic + "/includeLSLTimestamps", default=True)

        self.rawTopics_ROStoLSL = rospy.get_param(self.nodeTopic + "/" + config.param_MapsSubtopic +
                                                  "ROStoLSL", default=[])
        self.rawTopics_LSLtoROS = rospy.get_param(self.nodeTopic + "/" + config.param_MapsSubtopic +
                                                  "LSLtoROS", default=[])
        pass

    def InitLSLtoROS(self):
        rospy.loginfo("Map LSL to ROS:")

        if len(self.rawTopics_LSLtoROS) == 0:
            rospy.loginfo("Nothing")

        for map in self.rawTopics_LSLtoROS:
            self.mapper.append(Mapper_LSLtoROS(commonType=map["commonType"], topic=map["rostopic"],
                                               channelInfo=map["lslChannelInfo"], cyclicMode=self.cyclicMode,
                                               useLSLTypesBidirectional=self.useLSLTypesBidirectional,
                                               includeLSLTimestamps=self.includeLSLTimestamps))

            rospy.loginfo("(LSL)" + self.mapper[-1].lslTopic + " -> " + self.mapper[-1].topic + "(ROS)" + " Type: " +
                          str(self.mapper[-1].converter.rosType))

    def InitROStoLSL(self):
        rospy.loginfo("Map ROS to LSL:")

        if len(self.rawTopics_ROStoLSL) == 0:
            rospy.loginfo("Nothing")

        for map in self.rawTopics_ROStoLSL:
            self.mapper.append(Mapper_ROStoLSL(commonType=map["commonType"], topic=map["rostopic"],
                                               channelInfo=map["lslChannelInfo"], cyclicMode=self.cyclicMode,
                                               useLSLTypesBidirectional=self.useLSLTypesBidirectional,
                                               includeLSLTimestamps=self.includeLSLTimestamps))

            rospy.loginfo("(ROS)" + self.mapper[-1].topic + " -> " + self.mapper[-1].lslTopic + "(LSL)" + " Type: "
                          + str(self.mapper[-1].converter.lslType))




    def CollectData(self):
        jobs = []
        for mapper in self.mapper:
            jobs.append(mapper.CollectData)

        self.RunThreaded(jobs)

    def UpdateData(self):
        jobs = []
        for mapper in self.mapper:
            jobs.append(mapper.UpdateData)

        self.RunThreaded(jobs)

    def RunThreaded(self, jobs):
        threads = []
        for job in jobs:
            thread = threading.Thread(target=job)
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

    def RunCyclicMode(self):
        rospy.loginfo("Started " + config.nodeName + " in CyclicMode")
        while not rospy.is_shutdown():
            #first create image of all channels and ros topics
            self.CollectData()
            #update all channels from the LSL and update Stuff (in parallell threads)
            self.UpdateData()

            #self.rate.sleep()

    def RunEventBasedMode(self):
        rospy.loginfo("Started " + config.nodeName + " in EventBasedMode")
        while not rospy.is_shutdown():
            rospy.spin()

    def node(self):

        self.Init()

        self.rate = rospy.Rate(config.rate)

        if self.cyclicMode:
            self.RunCyclicMode()
        else:
            self.RunEventBasedMode()

