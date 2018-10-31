import rospy
from abc import ABCMeta, abstractmethod
from MessageConverter import MessageConverter

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


    def __init__(self, commonType, topic, channelInfo, cyclicMode):
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
        self.messageConverter = MessageConverter()
        self.conversion = self.messageConverter.FindConversion(commonType)


    @abstractmethod
    def CollectData(self):
        pass

    @abstractmethod
    def UpdateData(self):
        pass