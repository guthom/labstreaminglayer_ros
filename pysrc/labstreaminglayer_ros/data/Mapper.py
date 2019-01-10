import os, sys
import rospy
from abc import abstractmethod
import Converter
from pylsl import StreamInfo, StreamOutlet


projectDir = os.path.join(os.path.dirname(__file__), '../')
sys.path.append(projectDir + "nodes")
import nodeconfig as config

class Mapper(object):

    commonType = None
    topic = None
    lslTopic = None
    contentType = ""
    lslUID = ""
    lslFrequency = None
    publisher = None
    subscriber = None
    cyclicMode = False
    lastRosMsg = None
    lastCollectedRosMsg = None
    lastLslMsg = None
    lastCollectedLslMsg = None
    converter = None
    timeout = 0.1
    lslStreamInfo = None

    def __init__(self, commonType, topic, channelInfo, cyclicMode):
        self.converter = self.FindConverter(commonType)
        self.commonType = commonType
        self.topic = topic

        self.lslFrequency = int(channelInfo["frequency"])
        self.lslTopic = str(channelInfo["topic"])
        if channelInfo["UID"] != "":
            self.lslUID = channelInfo["UID"]

        if "contentType" in channelInfo:
            self.contentType = str(channelInfo["contentType"])

        self.eventBased = cyclicMode
        self.lastRosMsg = None
        self.lastLslMsg = None
        self.subscriber = None
        self.publisher = None
        self.timeout = config.lslTimeout

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

        lslStreamInfo = StreamInfo(name=self.lslTopic, type=self.contentType,
                                        channel_count=self.converter.lslChannels, nominal_srate=self.lslFrequency,
                                        channel_format=self.converter.lslType, source_id=self.lslUID)
        return lslStreamInfo

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
        if data is not None:
            return self.converter.ToROS(data)

    def ToLSL(self, data):
        if data is not None:
            return self.converter.ToLSL(data)

    def __del__(self):
        self.lslStreamInfo.__del__()
        self.publisher.close_stream()