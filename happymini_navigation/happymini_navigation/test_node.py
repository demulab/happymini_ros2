import os
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_prefix


class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")

        #params = ['tinpo', [10.0, 20.0, 30.0]]
        #self.declare_parameters('Onishi', [params])
        #print(self.has_parameter('Onishi'))
        # パラメータをセット
        dir_name = 'happymini_ros2'
        package_name = 'happymini_navigation'
        package_path = get_package_prefix(package_name)
        location_path = os.path.join(os.path.dirname(
            os.path.dirname(package_path)), 'src', 'happymini_ros2', package_name, 'location')
        map_path = os.path.join(os.path.dirname(
            os.path.dirname(package_path)), 'src', 'happymini_ros2', package_name, 'maps')
        print(package_path)
        print(location_path)
        print(map_path)
        params = [
                ['Onishi', [10, 20]],
                ['Makima', [721.5, 50.1]]
                ]
        self.declare_parameters('Onishi', params)
        #print(self.has_parameter('Onishi.Makima'))
        # 新しいパラメータをセット
        get_params = self.get_parameter('Onishi.Makima').value
        print(get_params)
        new_param = Parameter('Onishi.Makima', Parameter.Type.from_parameter_value([777.7, 555.5]), [777.7, 555.5])
        self.set_parameters([new_param])
        print(self.get_parameter('Onishi.Makima').value)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
