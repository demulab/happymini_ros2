import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import smach
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
# Module
from happymini_manipulation.motor_controller import JointController
# Custom msg
from happymini_msgs.action import GraspBag
from happymini_msgs.srv import TextToSpeech, SpeechToText


class DetectPose(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['detected', 'time_out'],
                output_keys=['left_right_out'])
        # Node
        self.node = node
        self.logger = node.get_logger()
        # Topic
        self.node.create_subscription(String, 'way', self.pose_callback, 10)
        # Value
        self.hand_pose = 'None pose'
        self.timeout_flg = False

    def pose_callback(self, receive_msg):
        self.hand_pose = receive_msg.data

    def get_pose(self, timeout=10):
        time_count = 0.0
        tmp_time = 0.0
        start_time = time.time()
        # Get pose
        while self.hand_pose == 'None pose' and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.5)
            # Time count
            time_count = time.time() - start_time
            one_sec = time_count - tmp_time
            if time_count >= timeout:
                self.logger.info("Time out !!!")
                self.timeout_flg = True
                break
            # 1秒ごとに出力
            if one_sec >= 1.0:
                self.logger.info(f"hand_pose is <{self.hand_pose}>")
                tmp_time = time_count

    def execute(self, userdata):
        self.get_pose()
        userdata.left_right_out = self.hand_pose
        if self.timeout_flg:
            return 'time_out'
        return 'detected'


class GraspBagState(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['failed', 'success'],
                input_keys=['left_right_in'])
        # Node
        self.node = node
        self.logger = node.get_logger()
        # Action
        self.grasp_bag = ActionClient(node, GraspBag, 'grasp_bag_server')
        # Value
        self.result = False

    def send_goal(self, left_right):
        goal_msg = GraspBag.Goal()
        goal_msg.left_right = left_right
        goal_msg.coordinate = [0.25, 0.4]
        self.grasp_bag.wait_for_server()
        self.send_goal_future = self.grasp_bag.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.error("Goal rejected")
            return 'failed'
        self.logger.info("Goal approved !")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.result = future.result().result.result
        self.logger.info(f"Result: {self.result}")

    def feedback_callback(self, feedback_msg):
        self.logger.info(feedback_msg.feedback.state)

    def execute(self, userdata):
        self.logger.info(f"(left_right, <{userdata.left_right_in}>)")
        self.send_goal(userdata.left_right_in)
        while not self.result:
            rclpy.spin_once(self.node, timeout_sec=0.5)
        return 'success'


class Chaser(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['stop'])
        # Node
        self.node = node
        self.logger = node.get_logger()
        # Topic
        self.follow_pub = self.node.create_publisher(String, 'follow_human', 10)
        self.twist_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.node.create_subscription(Bool, 'find_human', self.find_human_callback, 10)
        self.node.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.node.create_subscription(Bool, 'chaser_check', self.chaser_check_callback, 10)
        # Timer
        self.timer = self.node.create_timer(0.01, self.timer_callback)
        # Value
        self.chaser_msg = String()
        self.twist_value = Twist()
        self.sub_linear_x = 999.9
        self.sub_angular_z = 999.9
        self.find_human_flg = False
        self.chaser_check_flg = False

    def timer_callback(self):
        self.follow_pub.publish(self.chaser_msg)

    def find_human_callback(self, receive_msg):
        self.find_human_flg = receive_msg.data

    def cmd_vel_callback(self, receive_msg):
        self.sub_linear_x = receive_msg.linear.x
        self.sub_angular_z = receive_msg.angular.z

    def chaser_check_callback(self, receive_msg):
        self.chaser_check_flg = receive_msg.data

    def execute(self, userdata):
        self.chaser_msg.data = 'start'
        start_time = 0.0
        time_counter = 0.0
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.sub_linear_x == 0.0 and self.chaser_check_flg and time_counter > 5.0:
                self.logger.info("Is this the location of a car?")
                _ = self.node.tts('Is this the location of a car?')
                self.logger.info("Doing /stt")
                response = self.node.stt(cmd='yes_no')
                if response == 'yes':
                    break
                self.logger.info(f"{response}")
            elif self.sub_linear_x == 0.0 and start_time != 0.0:
                time_counter = time.time() - start_time
            elif self.sub_linear_x == 0.0 and time_counter == 0.0:
                start_time = time.time()
            else:
                time_counter = 0.0
                start_time = 0.0
            self.logger.info(f"{time_counter}")               
        # 止まるまで待つ
        self.logger.info("Stopping Chaser ...")
        while self.chaser_check_flg and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            self.chaser_msg.data = 'stop'
        return 'stop'


class GiveBag(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['finished'])
        # Node
        self.node = node
        # Module
        self.arm = JointController()

    def execute(self, userdata):
        self.arm.give()
        self.node.tts('Here you are.')
        time.sleep(5.0)
        self.arm.start_up()
        self.node.tts('No problem.')
        return 'finished'


class Return(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['success'])
        self.node = node

    def execute(self, userdata):
        self.node.tts('Fuckin bich')
        return 'success'


class CarryMyLuggage(Node):
    def __init__(self):
        super().__init__('carry_my_luggage')
        # Service
        self.tts_srv = self.create_client(TextToSpeech, 'tts')
        self.stt_srv = self.create_client(SpeechToText, 'stt')
        while not self.tts_srv.wait_for_service(timeout_sec=1.0) and not self.stt_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/tts and /stt is not here ...")
        self.tts_msg = TextToSpeech.Request()
        self.stt_msg = SpeechToText.Request()

    def tts(self, text):
        # msg
        self.tts_msg.text = text
        # Request
        tts_srv_future = self.tts_srv.call_async(self.tts_msg)
        while not tts_srv_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if tts_srv_future.result() is not None:
            return True
        else:
            self.get_logger().info("/tts service call failed")
            return False

    def stt(self, cmd):
        # msg
        self.stt_msg.cmd = cmd
        # Request
        stt_srv_future = self.stt_srv.call_async(self.stt_msg)
        while not stt_srv_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if stt_srv_future.result() is not None:
            return stt_srv_future.result().result
        else:
            self.get_logger().info("/stt service call failed")
            return False

    def execute(self):
        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['end'])
        sm.userdata.left_right = 'NULL'
        # Open the container
        with sm:
            # Add states
            smach.StateMachine.add(
                    'DETECT_POSE', DetectPose(self),
                    transitions={'time_out':'DETECT_POSE',
                                 'detected':'GRASP_BAG'},
                    remapping={'left_right_out':'left_right'})
            smach.StateMachine.add(
                    'GRASP_BAG', GraspBagState(self),
                    transitions={'failed':'GRASP_BAG',
                                 'success':'CHASER'},
                    remapping={'left_right_in':'left_right'})
            smach.StateMachine.add(
                    'CHASER', Chaser(self),
                    transitions={'stop':'GIVE_BAG'})
            smach.StateMachine.add(
                    'GIVE_BAG', GiveBag(self),
                    transitions={'finished':'RETURN'})
            smach.StateMachine.add(
                    'RETURN', Return(self),
                    transitions={'success':'end'})
        # Execute
        outcome = sm.execute()
        self.get_logger().info(f"outcome: {outcome}")


def main():
    rclpy.init()
    cml = CarryMyLuggage()
    cml.execute()
