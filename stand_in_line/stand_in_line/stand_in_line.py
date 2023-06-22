import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class StandInLineServer(Node):
    def __init__(self):
        super().__init__('stand_in_line_node')
        # Subscriber
        self.yolo_sub = self.create_subscription(Float32MultiArray, 'topic', self.yolo_callback, 10)
        # Value
        self.xx = 0.0
        self.zz = 0.0

    def yolo_callback(self, receive_msg):
        print(receive_msg)
        self.xx = receive_msg.data[0]
        self.zz = receive_msg.data[2]


def main():
    rclpy.init()
    stand_line_node = StandInLineServer()
    try:
        rclpy.spin(stand_line_node)
    except KeyboardInterrupt:
        pass
    stand_line_node.destroy_node()
    rclpy.shutdown()
