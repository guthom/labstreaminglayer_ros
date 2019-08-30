from Mapper import Mapper
import rospy
from pylsl import StreamInlet, resolve_stream
class Mapper_LSLtoROS(Mapper):

    lslStreamInlet = None
    def __init__(self, commonType, topic, channelInfo, cyclicMode):
        super(Mapper_LSLtoROS, self).__init__(commonType, topic, channelInfo, cyclicMode)
        self.publisher = rospy.Publisher(topic, self.converter.rosType, queue_size=10)
        streams = resolve_stream('name', self.lslTopic)[0]
        #self.lslStreamInlet = StreamInlet(self.lslStreamInfo)
        self.lslStreamInlet = StreamInlet(streams)
        self.counter = 0

    def CollectData(self):
        self.lastCollectedLslMsg = self.lslStreamInlet.pull_sample()#timeout=self.timeout)
        self.counter +=1
        print str(self.counter)
        if self.lastCollectedLslMsg[0] is None:
            self.lastCollectedLslMsg = None

    def UpdateData(self):
        if self.lastCollectedLslMsg is not None and self.publisher is not None:
            self.publisher.publish(self.ToROS(self.lastCollectedLslMsg))