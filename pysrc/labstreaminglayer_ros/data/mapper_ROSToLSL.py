import sys, os
import rospy
from mapperBase import MapperBase

from importlib import import_module

from pylsl import StreamInfo, StreamOutlet
projectDir = os.path.join(os.path.dirname(__file__), '../')

class Mapper_ROStoLSL(MapperBase):

    binarySubscriber = None
    lslStreamInfo = None

    def __init__(self, commonType, topic, channelInfo, cyclicMode):
        super(Mapper_ROStoLSL, self).__init__(commonType, topic, channelInfo, cyclicMode)

        self.subscriber = rospy.Subscriber(self.topic, self.messageConverter.GetRosType(commonType),
                                           self.SubscriberCallback)

        self.lslStreamInfo = self.messageConverter.GetLSLStreamInfo(self.commonType, self.channelTopic, self.contentType)
        self.publisher = StreamOutlet(self.lslStreamInfo)

    def SubscriberCallback(self, data):

        self.lastRosMsg = data

        if not self.cyclicMode:
            self.CollectData()
            self.UpdateData()

    def CollectData(self):
        self.lastCollectedRosMsg = self.lastRosMsg

    def UpdateData(self):
        if self.lastCollectedRosMsg is not None and self.publisher is not None:
            self.publisher.push_sample(self.conversion.ToLSL(self.lastCollectedRosMsg))
        pass

