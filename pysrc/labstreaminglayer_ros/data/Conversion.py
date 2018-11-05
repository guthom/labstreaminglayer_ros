import rospy
import os
import importlib
import Converter
from pylsl import StreamInfo

from importlib import import_module

projectDir = os.path.join(os.path.dirname(__file__), '../')

class Conversion(object):

    commonType = ""
    rawRosType = ""
    lslChannelInfo = ""

    rosType = None
    frequency = 30

    def __init__(self, commonType, rosType, lslChannelInfo):
        self.commonType = commonType
        self.rawRosType = rosType
        self.lslChannelInfo = lslChannelInfo

        name = rosType.split('/')
        ros_pkg = name[0] + '.msg'
        msg_type = name[1]

        self.rosType = getattr(import_module(ros_pkg), msg_type)
        pass

    @staticmethod
    def Get_Converter(name):
        converter = getattr(Converter, "Float32")
        
        if converter is None:
            rospy.logwarn("Can not find converter for common type: " + name + " Will load BaseConverter Instead! " +
                          "Please implement new ConverterClass in data/Converter.py!")
            converter = getattr(Converter, "ConverterBase")

        return converter

    def GetStreamInfo(self, streamName, contentType = ""):
        self.lslStreamInfo = StreamInfo(streamName, str(contentType),
                                        self.lslChannelInfo["channels"], self.frequency,
                                        str(self.lslChannelInfo["type"]))
        return self.lslStreamInfo

    def GetRosType(self):
        return self.rosType

    def ToROS(self, data):
        pass

    def ToLSL(self, data):
        converter = self.Get_Converter(self.commonType)
        return converter.ToLSL(data)



