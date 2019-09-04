import sys, os
import rospy
from Mapper import Mapper

from pylsl import StreamOutlet
projectDir = os.path.join(os.path.dirname(__file__), '../')

class Mapper_ROStoLSL(Mapper):
    def __init__(self, commonType, topic, channelInfo, cyclicMode, useLSLTypesBidirectional, includeLSLTimestamps):

        self.binarySubscriber = None
        self.lslStreamInfo = None

        super(Mapper_ROStoLSL, self).__init__(commonType, topic, channelInfo, cyclicMode, useLSLTypesBidirectional,
                                              includeLSLTimestamps)

        if useLSLTypesBidirectional is False and self.converter.rosStdType:
            self.subscriber = rospy.Subscriber(self.topic, self.converter.rosStdType, self.SubscriberCallback)
        else:
            self.subscriber = rospy.Subscriber(self.topic, self.converter.rosType, self.SubscriberCallback)

        self.publisher = StreamOutlet(self.lslStreamInfo)

    def SubscriberCallback(self, data):

        self.lastRosMsg = data

        if not self.cyclicMode:
            self.CollectData()
            self.UpdateData()

    def CollectData(self):
        self.lastCollectedRosMsg = self.lastRosMsg
        self.lastRosMsg = None

    def UpdateData(self):
        if self.lastCollectedRosMsg is not None and self.publisher is not None:
            self.publisher.push_sample(self.ToLSL(self.lastCollectedRosMsg))
            self.lastCollectedRosMsg = None
        pass

    def __del__(self):
        self.lslStreamInfo.__del__()
        self.publisher.close_stream()


