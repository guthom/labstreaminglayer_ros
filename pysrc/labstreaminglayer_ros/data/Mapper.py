import os, json
import rospy
from abc import abstractmethod
import Converter
from pylsl import StreamInfo, StreamOutlet

class Mapper(object):

    commonType = None
    topic = None
    channelTopic = None
    contentType = ""
    frequency = None
    publisher = None
    subscriber = None
    cyclicMode = False
    lastRosMsg = None
    lastCollectedRosMsg = None
    lastLslMsg = None
    lastCollectedLslMsg = None
    converter = None

    def __init__(self, commonType, topic, channelInfo, cyclicMode):
        self.converter = self.FindConverter(commonType)
        self.commonType = commonType
        self.topic = topic

        self.frequency = int(channelInfo["frequency"])
        self.channelTopic = str(channelInfo["topic"])

        if "contentType" in channelInfo:
            self.contentType = str(channelInfo["contentType"])

        self.eventBased = cyclicMode
        self.lastRosMsg = None
        self.lastLslMsg = None
        self.subscriber = None
        self.publisher = None


        self.lslStreamInfo = self.GetLSLStreamInfo()


    @abstractmethod
    def CollectData(self):
        pass

    @abstractmethod
    def UpdateData(self):
        pass

    def GetRosType(self):
        return self.converter.rosType

    def GetLSLStreamInfo(self,):
        self.lslStreamInfo = StreamInfo(name=self.channelTopic, type=self.contentType,
                                        channel_count=self.converter.lslChannels, nominal_srate=self.frequency,
                                        channel_format=self.converter.lslType)
        return self.lslStreamInfo

    @staticmethod
    def FindConverter(name):
        converter = getattr(Converter, name)
        converter = getattr(converter, name)

        if converter is None:
            rospy.logwarn("Can not find converter for common type: " + name + " Will load BaseConverter Instead! " +
                          "Please implement new ConverterClass in data/Converter.py!")
            converter = getattr(Converter, "ConverterBase")
            converter = getattr(converter, "ConverterBase")

        return converter()

    def ToROS(self, data):
        pass

    def ToLSL(self, data):
        return self.converter.ToLSL(data)