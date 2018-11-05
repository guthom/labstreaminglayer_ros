from ConverterBase import ConverterBase


class Transform(ConverterBase):

    commonType = "Transform"
    rosType = "geometry_msgs/Transform"
    lslChannels = 7
    lslType = "float32"

    @staticmethod
    def ToLSL(data):
        translation = data.transform.translation
        rotation = data.transform.rotation
        return [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w]
