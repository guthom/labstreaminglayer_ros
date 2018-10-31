import os, sys, json
from pylsl import StreamInfo

from importlib import import_module

projectDir = os.path.join(os.path.dirname(__file__), '../')

class Conversion(object):

    rawRosType = ""
    lslChannelInfo = ""

    rosType = None
    frequency = 30

    def __init__(self, rosType, lslChannelInfo):
        self.rawRosType = rosType
        self.lslChannelInfo = lslChannelInfo

        name = rosType.split('/')
        ros_pkg = name[0] + '.msg'
        msg_type = name[1]

        self.rosType = getattr(import_module(ros_pkg), msg_type)
        pass

    def GetStreamInfo(self, streamName, contentType = ""):
        self.lslStreamInfo = StreamInfo(streamName, str(contentType),
                                        self.lslChannelInfo["channels"], self.frequency,
                                        str(self.lslChannelInfo["type"]))
        return self.lslStreamInfo

    def GetRosType(self):
        return self.rosType

    def ToROS(self, lslStream):
        pass

    def ToLSL(self, rosMSG):
        return [rosMSG.data]


class MessageConverter(object):

    rawConversions = None
    conversions = None


    def __init__(self):
        self.LoadConversions()
        pass


    def LoadConversions(self):
        with open(os.path.dirname(__file__) + "/Conversions.json") as data_file:
            self.rawConversions = json.load(data_file, encoding="utf-8")

        self.conversions = dict()
        for conversion in self.rawConversions:
            self.conversions[conversion["commonType"]] = Conversion(rosType=conversion["rosType"],
                                                                    lslChannelInfo=conversion["lslStream"])

    @staticmethod
    def ROStoLSL(message):
        return message

    @staticmethod
    def LSLtoROS(message):
        return message

    def GetRosType(self, commonType):
        conversion = self.FindConversion(commonType)
        return conversion.rosType

    def FindConversion(self, commonType):
        return self.conversions[commonType]

    def GetLSLStreamInfo(self, commonType, streamName, contentType = ""):
        conversion = self.FindConversion(commonType)
        return conversion.GetStreamInfo(streamName, contentType)


