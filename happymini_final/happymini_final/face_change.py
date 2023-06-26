import rclpy
from rclpy.node import Node

import subprocess
import time
from happymini_msgs.srv import FaceChange 

class Face_Change_Subprocess(Node):
    def __init__(self):
        super().__init__('face_change_node')
        self.face = ["eog -f -g -w /home/demulab/mini_face/mini_face.gif", "eog -f -g -w /home/demulab/mini_face/robin.jpg"]
        self.create_service(FaceChange, 'face_change', self.call)


    def call(self, srv_req, srv_res):
        #print(srv_req.num)
        subprocess.Popen(self.face[srv_req.num].split())
        srv_res.result = True
        return srv_res

def main():
    rclpy.init()
    node = Face_Change_Subprocess()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
