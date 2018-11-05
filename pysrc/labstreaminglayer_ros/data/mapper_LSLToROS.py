from Mapper import Mapper
import rospy

class Mapper_LSLtoROS(Mapper):


    def __init__(self, commonType, topic, channel, cyclicMode):
        super(Mapper, self).__init__(commonType, topic, channel, cyclicMode)

        self.publisher = rospy.Publisher(topic, self.converter.rosType, queue_size=10)



    def CollectData(self):
        pass

    def UpdateData(self):
        self.publisher.publish(self.ToROS(self.lastCollectedRosMsg))
        pass