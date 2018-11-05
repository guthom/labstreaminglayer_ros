class ConverterBase(object):
    def __init__(self):
        pass

    @staticmethod
    def ToLSL(data):
        return [data]

    @staticmethod
    def ToROS(data):
        return data

class Image(ConverterBase):

    @staticmethod
    def ToLSL(data):
        return data

    @staticmethod
    def ToROS(data):
        return data

class Float32(ConverterBase):
    pass

class Bool(ConverterBase):
    pass