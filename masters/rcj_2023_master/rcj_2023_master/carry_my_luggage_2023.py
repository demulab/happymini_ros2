import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from happymini_msgs.srv import BagLocalization
from happymini_msgs.action import GraspBag
from airobot_interfaces.srv import StringCommand
import time
from happymini_manipulation.motor_controller import JointController
from happymini_navigation.navi_location import WayPointNavi
from grasp_bag.grasp_bag_server import GraspBagServer 
import sys
import pyttsx3
#text = None

def synthesis(text = None):
    # 音声合成を行うことをloggerで表示します．
    print(f'音声合成を実行します')
    print(f'発話内容は "{text}"')

    # 音声合成エンジンを初期化します．
    engine = pyttsx3.init()

    # 言語を設定します．
    lang = "en-US"
    engine.setProperty('voice', lang)
    # rateは、1分あたりの単語数で表した発話速度。基本は、200です。
    rate = engine.getProperty("rate")
    engine.setProperty("rate",160)

    # ボリュームは、0.0~1.0の間で設定します。
    volume = engine.getProperty('volume')
    engine.setProperty('volume',1.0)

    # テキストを音声に合成します．
    engine.say(text)

    engine.runAndWait()

class TestNode(Node):
    def __init__(self):
        super().__init__('test_grasp_node')
        # Action
        self.grasp_bag = GraspBagServer() #ActionClient(self, GraspBag, 'grasp_bag_server')
        # Topic
        self.create_subscription(String, 'way', self.hand_pose_callback, 10)
        # Value
        self.hand_pose = None

    def hand_pose_callback(self, receive_msg):
        self.hand_pose = receive_msg.data

    def send_goal(self, left_right):
        goal_msg = GraspBag.Goal()
        goal_msg.left_right = left_right
        goal_msg.coordinate = [0.25, 0.4]
        self.get_logger().info("panti")
        self.grasp_bag.wait_for_server()
        self.get_logger().info("tinti")
        result = self.grasp_bag.send_goal(goal_msg)
        self.get_logger().info("tin")
        print(result)
    #    goal_future = self.grasp_bag.send_goal_async(goal_msg)
    #    goal_future.add_done_callback(self.goal_response_callback)

    #def goal_response_callback(self, future):
    #    goal_handle = future.result()
    #    if not goal_handle.accepted:
    #        self.get_logger().info("ゴール拒否")
    #        return

    #    self.get_logger().info("ゴールおけまる")
    #    get_result_future = goal_handle.get_result_async()
    #    get_result_future.add_done_callback(self.get_result_callback)

    #def get_result_callback(self, future):
    #    result = future.result().result
    #    self.get_logger().info(f"結果: {result}")

    def execute(self):
        time.sleep(1.0)
        while self.hand_pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
            self.get_logger().info("hand_pose is empty ...")
        time.sleep(0.5)
        self.get_logger().info("unti")
        self.get_logger().info(f"hand_pose >>> {self.hand_pose}")
        self.grasp_bag.execute(self.hand_pose)
        #self.send_goal(self.hand_pose)
        #self.send_goal('right')
        self.get_logger().info("onti")


class Navigation(Node):
    def __init__(self):
        super().__init__('test_navigation')
        self.wp_node = WayPointNavi()
        self.count = 0

    def execute(self):
        #self.wp_node.set_params()
        self.wp_node.navigation_execute('start_cml')
        print(f'navicount:{self.count}')
        self.count = 1
        time.sleep(2)

    def execute2(self):
        self.wp_node.navigation_execute('start_fmm')
        time.sleep(2)
    
    def execute3(self):
        self.wp_node.navigation_execute('goal_car2')
        time.sleep(2)

        


class GiveBag(Node):
    def __init__(self):
        super().__init__('test_give_bag')
        self.jc_node = JointController()
        

    def execute(self):
        time.sleep(1.0)
        self.jc_node
        self.get_logger().info("give a bag")
        #self.send_goal()
        self.jc_node.manipulation([0.2, 0.5])
        time.sleep(5.0)
        self.jc_node.start_up()

class HitoSekkin(Node):
    def __init__(self):
        super().__init__('test_hitosekkin')
        self.pub = self.create_publisher(String, 'master', 10)

    def execute(self):
        msg = String()
        msg.data = 'start'
        self.pub.publish(msg)

class Speech(Node):
    def __init__(self):
        super().__init__('test_speech') 
        self.node = rclpy.create_node('cml_client')
        self.client = self.node.create_client(StringCommand, 'cml_speech_service/wake_up')
        self.req = None
        self.future = None
        
    def execute(self):
        time.sleep(1.0)
        print(f'count: {Navigation().count}')
        #if Navigation().count == 1:
        self.req = StringCommand.Request()
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self.node, self.future)
        
            
        if future.result() is not None:
            response = future.result()
            self.get_logger().info('応答: ' + response.answer)
        else:
            self.get_logger().info('サービスが応答しませんでした。')

def main():
    rclpy.init()
    synthesis("Start, carry my luggage.")
    tn = TestNode()
    gb = GiveBag()
    n = Navigation()
    hs = HitoSekkin()
    s = Speech()
    k = 0

    try:
        time.sleep(3)
        tn.execute()
        time.sleep(15)
        #print('ok')
        n.execute3()
        time.sleep(10)
        #s.execute()
        #time.sleep(10)
        gb.execute()
        time.sleep(10)
        n.execute()
        time.sleep(10)
        #hs.execute() 
        synthesis("Finish, carry my luggage.")

    except KeyboardInterrupt:
        pass
    finally:
        tn.destroy_node()
        rclpy.shutdown()
