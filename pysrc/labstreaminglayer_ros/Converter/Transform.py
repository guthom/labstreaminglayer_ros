from ConverterBase import ConverterBase
from labstreaminglayer_ros.msg import LSLTransform as message
from geometry_msgs.msg import Transform as stdmessage

class Transform(ConverterBase):

    def __init__(self):
        super(Transform, self).__init__(
            commonType="Transform",
            rosType=message,
            rosStdType=stdmessage,
            lslChannels=7,
            lslType="float32"
            )

    def ToLSL(self, data):
        translation = data.translation
        rotation = data.rotation
        return [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w]
