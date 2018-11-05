from ConverterBase import ConverterBase
from std_msgs.msg import Float32 as message

class Float32(ConverterBase):
    commonType = "float32"
    rosType = "std_msgs/Float32"
    lslChannels = 1
    lslType = "float32"

    @staticmethod
    def ToLSL(data):
        return [data.data]

    @staticmethod
    def ToROS(data):
        msg = message()
        msg.data = data[0][0]
        return msg