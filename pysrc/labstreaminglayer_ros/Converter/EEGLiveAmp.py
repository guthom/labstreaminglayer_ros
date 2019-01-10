from ConverterBase import ConverterBase
from std_msgs.msg import Float32MultiArray as message

class EEGLiveAmp(ConverterBase):
    commonType = "EEGLiveAmp"
    rosType = "std_msgs/Float32MultiArray"
    lslChannels = 32
    lslType = "float32"

    @staticmethod
    def ToLSL(data):
        return [data.data]

    @staticmethod
    def ToROS(data):
        msg = message()
        msg.data.extend(data[0])
        return msg