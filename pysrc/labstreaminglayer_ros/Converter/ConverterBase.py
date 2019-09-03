from importlib import import_module


class ConverterBase(object):

    def __init__(self, commonType, rosType, rosStdType, lslChannels, lslType):
        self.commonType = commonType
        self.rosType = rosType
        self.rosStdType = rosStdType
        self.lslChannels = lslChannels
        self.lslType = lslType

    def ExtractRosType(self, msgName):
        #used to extract message type from name string
        if msgName is None:
            return None
        name = msgName.split('/')
        ros_pkg = name[0] + '.msg'
        msg_type = name[1]
        return getattr(import_module(ros_pkg), msg_type)

    @staticmethod
    def ToLSL(data):
        raise Exception("ToLSL Method is not implemented yet! DO IT!")

    @staticmethod
    def ToROS(data):
        raise Exception("ToROS Method is not implemented yet! DO IT!")
