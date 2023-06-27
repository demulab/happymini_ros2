import tf2_msgs.msg
from  geometry_msgs.msg import TransformStamped

class HumanCoordGeneratorModule():
    def __init__(self):
        self.t = TransformStamped()

    def coord_generate(self, human_coord):
        self.t.header.frame_id = "camera_depth_frame"
        self.t.child_frame_id = "human_coord"
        self.t.transform.translation.x = human_coord.x
        self.t.transform.translation.x = human_coord.y
        self.t.transform.translation.x = human_coord.z = 0.0
        self.t.transform.rotation.x = 0.0
        self.t.transform.rotation.y = 0.0
        self.t.transform.rotation.z = 0.0
        self.t.transform.rotation.w = 1.0
        return self.t
