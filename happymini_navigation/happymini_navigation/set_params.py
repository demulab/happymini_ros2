import os
import sys
import time
import yaml
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node


class SetLocationParams(Node):
    def __init__(self):
        super().__init__("set_location_params_node")
        self.yaml_path = sys.argv[1]
        self.get_logger().info(f"Load the following YAML file: {self.yaml_path}")
        self.location_dict = {}
    
    def load_yaml(self):
        with open(self.yaml_path) as f:
            self.location_dict = yaml.safe_load(f)

    def set_params(self):
        self.load_yaml()
        # Set param
        location_params = [
                [key, value] for key, value in self.location_dict.items()
                ]
        self.declare_parameters('location_params', location_params)
        # Check param
        for key in self.location_dict.keys():
            param_name = 'location_params.' + key
            if self.has_parameter(param_name):
                self.get_logger().info(f"'{param_name}' already set")
            else:
                self.get_logger().error(f"Could not set '{param_name}'")
                break

def main():
    rclpy.init()
    slp_node = SetLocationParams()
    slp_node.set_params()
    rclpy.spin(slp_node)
    rclpy.shutdown()
