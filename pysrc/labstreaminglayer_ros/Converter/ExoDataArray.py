from ConverterBase import ConverterBase
from labstreaminglayer_ros.msg import LSLExoDataArray as message
from std_msgs.msg import Int32MultiArray as stdmessage

class ExoDataArray(ConverterBase):

    def __init__(self):
        super(ExoDataArray, self).__init__(
            commonType="ExoDataArray",
            rosType=message,
            rosStdType=stdmessage,
            lslChannels=8,
            lslType="int32"
            )