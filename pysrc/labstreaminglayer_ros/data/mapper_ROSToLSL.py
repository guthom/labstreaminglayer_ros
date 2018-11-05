import sys, os
import rospy
from Mapper import Mapper

from pylsl import StreamOutlet
projectDir = os.path.join(os.path.dirname(__file__), '../')

class Mapper_ROStoLSL(Mapper):

    binarySubscriber = None
    lslStreamInfo = None

    def __init__(self, commonType, topic, channelInfo, cyclicMode):
        super(Mapper_ROStoLSL, self).__init__(commonType, topic, channelInfo, cyclicMode)

        self.subscriber = rospy.Subscriber(self.topic, self.converter.rosType, self.SubscriberCallback)

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
            self.publisher.push_sample(self.ToLSL(self.lastCollectedRosMsg))
        pass

