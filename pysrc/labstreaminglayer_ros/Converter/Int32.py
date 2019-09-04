from ConverterBase import ConverterBase
from labstreaminglayer_ros.msg import LSLInt32 as message
from std_msgs.msg import Int32 as stdmessage

class Int32(ConverterBase):

    def __init__(self):
        super(Int32, self).__init__(
            commonType="int32",
            rosType=message,
            rosStdType=stdmessage,
            lslChannels=1,
            lslType="int32"
            )

