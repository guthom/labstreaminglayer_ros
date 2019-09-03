from ConverterBase import ConverterBase

from labstreaminglayer_ros.msg import LSLBool as message
from std_msgs.msg import Bool as stdmessage

class Bool(ConverterBase):

    def __init__(self):
        super(Bool, self).__init__(
            commonType="Bool",
            rosType=message,
            rosStdType=stdmessage,
            lslChannels=1,
            lslType="bool"
            )

    @staticmethod
    def ToLSL(data):
        return [data.data]

    @staticmethod
    def ToROS(data):
        msg = message()
        msg.data = data[0][0]
        msg.header.timestamp = data[1]
        return msg