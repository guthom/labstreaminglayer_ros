from Mapper import Mapper
import rospy
from pylsl import StreamInlet, resolve_stream
class Mapper_LSLtoROS(Mapper):

    lslStreamInlet = None
    def __init__(self, commonType, topic, channelInfo, cyclicMode, useLSLTypesBidirectional, includeLSLTimestamps):
        super(Mapper_LSLtoROS, self).__init__(commonType, topic, channelInfo, cyclicMode, useLSLTypesBidirectional,
                                              includeLSLTimestamps)

        if includeLSLTimestamps:
            self.publisher = rospy.Publisher(topic, self.converter.rosType, queue_size=10)
        else:
            self.publisher = rospy.Publisher(topic, self.converter.rosStdType, queue_size=10)

        print("Looking for LSLTopic: " + str(self.lslTopic))
        streams = resolve_stream('name', self.lslTopic)[0]
        print("Found LSLTopic: " + str(self.lslTopic))
        #self.lslStreamInlet = StreamInlet(self.lslStreamInfo)
        self.lslStreamInlet = StreamInlet(streams)
        self.counter = 0

    def CollectData(self):
        self.lastCollectedLslMsg = self.lslStreamInlet.pull_sample()
        if self.lastCollectedLslMsg[0] is None:
            self.lastCollectedLslMsg = None

    def UpdateData(self):
        if self.lastCollectedLslMsg is not None and self.publisher is not None:
            if self.includeLSLTimestamps:
                self.publisher.publish(self.ToROS(self.lastCollectedLslMsg))
            else:
                self.publisher.publish(self.ToROSStd(self.lastCollectedLslMsg))

    def __del__(self):
        pass