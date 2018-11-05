from ConverterBase import ConverterBase


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
        return data