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

    def ToLSL(self, data):
        return [data.data]

    def ToROSStd(self, data):
        msg = self.rosStdType()
        msg.data = data[0][0]
        return msg

    def ToROS(self, data):
        msg = self.rosType()
        msg.data = data[0][0]
        msg.header.timestamp = data[1]
        return msg
