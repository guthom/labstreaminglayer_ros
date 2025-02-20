from ConverterBase import ConverterBase
from labstreaminglayer_ros.msg import LSLFloat32 as message
from std_msgs.msg import Float32 as stdmessage

class Float32(ConverterBase):

    def __init__(self):
        super(Float32, self).__init__(
            commonType="float32",
            rosType=message,
            rosStdType=stdmessage,
            lslChannels=1,
            lslType="float32"
            )