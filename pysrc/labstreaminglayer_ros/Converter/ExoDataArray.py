from ConverterBase import ConverterBase
from std_msgs.msg import Int32MultiArray as message
from sensor_msgs.msg import Range as rangeMessage

class ExoDataArray(ConverterBase):
    commonType = "ExoDataArray"
    rosType = "std_msgs/Int32MultiArray"
    lslChannels = 8
    lslType = "int32"

    @staticmethod
    def ToLSL(data):
        return [data.data]

    @staticmethod
    def ToROS(data):
        msg = message()
        msg.data = data[0]
        return msg