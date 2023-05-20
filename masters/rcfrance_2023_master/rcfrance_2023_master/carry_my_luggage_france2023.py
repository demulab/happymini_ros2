import time
import rclpy
from rclpy.node import Node
import smach
from std_msgs.msg import String


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

class GraspBag(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['failed', 'success'],
                input_keys=['left_right_in'])
        self.node = node
        self.logger = node.get_logger()

    def execute(self, userdata):
        self.logger.info(f"(left_right, <{userdata.left_right_in}>)")
        return 'success'


class Chaser(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['stop'])

    def execute(self, userdata):
        return 'stop'


class VoiceRecognition(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['None', 'stop_follow'])

    def execute(self, userdata):
        return 'stop_follow'


class GiveBag(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['finished'])

    def execute(self, userdata):
        return 'finished'


class Return(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['success'])

    def execute(self, userdata):
        return 'success'


class CarryMyLuggage(Node):
    def __init__(self):
        super().__init__('carry_my_luggage')

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
                    'GRASP_BAG', GraspBag(self),
                    transitions={'failed':'GRASP_BAG',
                                 'success':'CHASER'},
                    remapping={'left_right_in':'left_right'})
            smach.StateMachine.add(
                    'CHASER', Chaser(self),
                    transitions={'stop':'VOICE_RECOGNITION'})
            smach.StateMachine.add(
                    'VOICE_RECOGNITION', VoiceRecognition(self),
                    transitions={'None':'CHASER',
                                 'stop_follow':'GIVE_BAG'})
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
