from ConverterBase import ConverterBase


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
        return data