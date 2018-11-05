from ConverterBase import ConverterBase

from std_msgs.msg import Bool as message

class Bool(ConverterBase):

    commonType = "Bool"
    rosType = "geometry_msgs/Bool"
    lslChannels = 1
    lslType = "bool"


    @staticmethod
    def ToLSL(data):
        return [data.data]

    @staticmethod
    def ToROS(data):
        msg = message()
        msg.data = data[0][0]
        return msg