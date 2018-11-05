from importlib import import_module


class ConverterBase(object):
    commonType = ""
    rosType = ""
    lslChannels = 0
    lslType = ""

    def __init__(self):
        self.ExtractRosType()

    def ExtractRosType(self):
        name = self.rosType.split('/')
        ros_pkg = name[0] + '.msg'
        msg_type = name[1]
        self.rosType = getattr(import_module(ros_pkg), msg_type)

    @staticmethod
    def ToLSL(data):
        raise Exception("ToLSL Method is not implemented yet! DO IT!")

    @staticmethod
    def ToROS(data):
        raise Exception("ToROS Method is not implemented yet! DO IT!")
