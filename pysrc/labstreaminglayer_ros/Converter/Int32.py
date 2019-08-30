from ConverterBase import ConverterBase
from std_msgs.msg import Float32 as message

class Int32(ConverterBase):
    commonType = "int32"
    rosType = "std_msgs/Int32"
    lslChannels = 1
    lslType = "int32"

    @staticmethod
    def ToLSL(data):
        return [data.data]

    @staticmethod
    def ToROS(data):
        msg = message()
        msg.data = data[0][0]
        return msg