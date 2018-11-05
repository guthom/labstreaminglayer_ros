import os, json
from abc import abstractmethod
from Conversion import Conversion

class MapperBase(object):

    commonType = None
    topic = None
    channelTopic = None
    contentType = None
    publisher = None
    subscriber = None
    cyclicMode = False
    lastRosMsg = None
    lastCollectedRosMsg = None
    lastLslMsg = None
    lastCollectedLslMsg = None
    messageConverter = None
    conversion = None
    rawConversions = None
    conversions = None

    def __init__(self, commonType, topic, channelInfo, cyclicMode):
        self.LoadConversions()
        self.commonType = commonType
        self.topic = topic
        self.channelTopic = channelInfo["topic"]

        if "contentType" in channelInfo:
            self.contentType = channelInfo["contentType"]

        self.eventBased = cyclicMode
        self.lastRosMsg = None
        self.lastLslMsg = None
        self.subscriber = None
        self.publisher = None
        self.conversion = self.FindConversion(commonType)


    @abstractmethod
    def CollectData(self):
        pass

    @abstractmethod
    def UpdateData(self):
        pass

    def LoadConversions(self):
        with open(os.path.dirname(__file__) + "/Conversions.json") as data_file:
            self.rawConversions = json.load(data_file, encoding="utf-8")

        self.conversions = dict()
        for conversion in self.rawConversions:
            self.conversions[conversion["commonType"]] = Conversion(commonType=conversion["commonType"],
                                                                    rosType=conversion["rosType"],
                                                                    lslChannelInfo=conversion["lslStream"])

    def GetRosType(self, commonType):
        conversion = self.FindConversion(commonType)
        return conversion.rosType

    def FindConversion(self, commonType):
        return self.conversions[commonType]

    def GetLSLStreamInfo(self, commonType, streamName, contentType = ""):
        conversion = self.FindConversion(commonType)
        return conversion.GetStreamInfo(streamName, contentType)
