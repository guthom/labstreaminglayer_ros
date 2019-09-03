from ConverterBase import ConverterBase
from labstreaminglayer_ros.msg import LSLEEGLiveAmp as message
from std_msgs.msg import Float32MultiArray as stdmessage

class EEGLiveAmp(ConverterBase):

    def __init__(self):
        super(EEGLiveAmp, self).__init__(
            commonType="EEGLiveAmp",
            rosType=message,
            rosStdType=stdmessage,
            lslChannels=32,
            lslType="float32"
            )

    @staticmethod
    def ToLSL(data):
        return [data.data]

    @staticmethod
    def ToROS(data):
        msg = message()
        msg.data.extend(data[0])
        msg.header.timestamp = data[1]
        return msg