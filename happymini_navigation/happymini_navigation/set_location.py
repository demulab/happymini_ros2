import os
import time
import yaml
import subprocess as sp
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_prefix
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from happymini_msgs.srv import SetLocation


class SetLocationServer(Node):
    def __init__(self):
        super().__init__('set_location_node')
        self.srv = self.create_service(SetLocation, 'set_location_server', self.checkState)
        self.get_logger().info("Ready to set_location_server")

        # Value
        self.tf_buffer = Buffer()
        self.tf = TransformListener(self.tf_buffer, self)
        self.location_dict = {}
        self.location_pose_x = 0.00
        self.location_pose_y = 0.00
        self.location_pose_z = 0.00
        self.location_pose_w = 0.00

    def getMapPosition(self):
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            self.location_pose_x = trans.transform.translation.x
            self.location_pose_y = trans.transform.translation.y
            self.location_pose_z = trans.transform.rotation.z
            self.location_pose_w = trans.transform.rotation.w
        except Exception as e:
            self.get_logger().error(str(e))

    def checkState(self, req, res):
        if req.state == 'add':
            self.get_logger().info("Add location")
            res.result = self.addLocation(req.name)
        elif req.state == 'save':
            self.get_logger().info("Save location")
            res.result = self.saveLocation(req.name)
        else:
            self.get_logger().error("<" + req.state + "> state doesn't exist.")
            res.result = False
        return res

    def addLocation(self, name):
        if name in self.location_dict:
            self.get_logger().error('<' + name + '> has been registerd. Please enter a different name.')
            return False
        elif name == '':
            self.get_logger().error("No location name enterd.")
            return False
        else:
            self.getMapPosition()
            self.location_dict[name] = []
            self.location_dict[name].append(self.location_pose_x)
            self.location_dict[name].append(self.location_pose_y)
            self.location_dict[name].append(self.location_pose_z)
            self.location_dict[name].append(self.location_pose_w)
            for location_data in self.location_dict:
                self.get_logger().info(location_data)
            self.get_logger().info(f"Registered <{name}>")
            return True

    def saveLocation(self, file_name):
        try:
            # パラメータをセット
            location_params = [
                    [key, value] for key, value in self.location_dict.items()
                    ]
            self.declare_parameters('location_params', location_params)
            # YAML
            dir_name = 'happymini_ros2'
            package_name = 'happymini_navigation'
            package_path = get_package_prefix(package_name)
            location_path = os.path.join(os.path.dirname(
                os.path.dirname(package_path)), 'src', 'happymini_ros2', package_name, 'location/')
            map_path = os.path.join(os.path.dirname(
                os.path.dirname(package_path)), 'src', 'happymini_ros2', package_name, 'maps/')
            with open(location_path + file_name + '.yaml', 'w') as f:
                yaml.dump(self.location_dict, f)
            os.makedirs(map_path, exist_ok=True)
            # MAP
            sp.Popen(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path + file_name])
            self.get_logger().info(f"Saved as <{file_name}>")
            return True
        except Exception as e:
            self.get_logger().error("Could not save. Reason: " + str(e))
            return False


def main(args=None):
    rclpy.init(args=args)
    set_location_server = SetLocationServer()
    rclpy.spin(set_location_server)
    set_location_server.destroy_node()
    rclpy.shutdown()
