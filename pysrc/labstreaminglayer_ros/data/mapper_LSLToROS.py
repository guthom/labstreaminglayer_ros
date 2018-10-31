from mapperBase import MapperBase
import rospy

class Mapper_LSLtoROS(MapperBase):


    def __init__(self, commonType, topic, channel, cyclicMode):
        super(MapperBase, self).__init__(commonType, topic, channel, cyclicMode)

        self.publisher = rospy.Publisher(topic, self.messageConverter.GetRosType(commonType), queue_size=10)



    def CollectData(self):
        pass

    def UpdateData(self):
        self.publisher.publish(self.conversion.ToROS(self.lastCollectedRosMsg))
        pass