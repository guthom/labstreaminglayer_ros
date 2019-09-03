from Transform import Transform
from labstreaminglayer_ros.msg import LSLTransformStamped as message
from geometry_msgs.msg import TransformStamped as stdmessage

class TransformStamped(Transform):

    def __init__(self):
        super(Transform, self).__init__(
            commonType="TransformStamped",
            rosType=message,
            rosStdType=stdmessage,
            lslChannels=7,
            lslType="float32"
            )

    @staticmethod
    def ToLSL(data):
        translation = data.transform.translation
        rotation = data.transform.rotation
        return [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w]
