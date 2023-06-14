import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import smach
import smach_ros
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
# Module
from happymini_manipulation.motor_controller import JointController
# Custom msg
from happymini_msgs.action import GraspBag
from happymini_msgs.srv import NaviLocation, SpeechToText, WavPlay


class EnterRoom(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['success'])
        # Node
        self.node = node
        self.logger = node.get_logger()

    def execute(self, userdata):
        time.sleep(1.0)
        return 'success'


class Navigation(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['wait_position', 'party_room'],
                input_keys=['location_name_in'])
        # Node
        self.node = node
        self.logger = node.get_logger()
        # Service
        #self.navi = node.create_client(NaviLocation, 'navi_location_server')
        #while not self.navi.wait_for_service(timeout_sec=1.0) and rclpy.ok():
        #    self.logger.info("/navi_location_server is not here ...")
        self.req = NaviLocation.Request()

    def do_navigation(self):
        # Call
        future = self.navi.call_async(self.req)
        # Waiting
        while not future.done() and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        # Result
        if future.result() is not None:
            navi_result = future.result().result
            print(navi_result)
        else:
            self.logger.info(f"Service call failed")

    def execute(self, userdata):
        self.req.location_name = 'recp_' + userdata.location_name_in
        time.sleep(1.0)
        #navi_result = False
        #counter = 1
        #while not navi_result and rclpy.ok():
        #    if counter > 3:
        #        break
        #    navi_result = self.do_navigation()
        #    counter += 1
        return userdata.location_name_in


class WaitGuest(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['success'])
        # Node
        self.node = node
        self.logger = node.get_logger()

    def execute(self, userdata):
        time.sleep(1.0)
        return 'success'


class GetInfo(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['success', 'secound'],
                input_keys=['guest_num_in'],
                output_keys=['location_name_out'])
        # Node
        self.node = node
        self.logger = node.get_logger()

    def execute(self, userdata):
        time.sleep(1.0)
        userdata.location_name_out = 'party_room'
        if userdata.guest_num_in == 1:
            return 'success'
        elif userdata.guest_num_in == 2:
            return 'secound'
        else:
            pass


class GetFeature(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['success'])
        # Node
        self.node = node
        self.logger = node.get_logger()

    def execute(self, userdata):
        time.sleep(1.0)
        return 'success'


class SayInfo(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['success'],
                input_keys=['guest_num_in'])
        # Node
        self.node = node
        self.logger = node.get_logger()

    def execute(self, userdata):
        # 2人目の特徴を言うメソッド追加予定
        time.sleep(1.0)
        return 'success'


class GuideToSeat(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['success', 'secound'],
                input_keys=['guest_num_in'],
                output_keys=['guest_num_out', 'location_name_out'])
        # Node
        self.node = node
        self.logger = node.get_logger()

    def execute(self, userdata):
        time.sleep(1.0)
        userdata.location_name_out = 'wait_position'
        if userdata.guest_num_in == 1:
            userdata.guest_num_out = userdata.guest_num_in + 1
            return 'success'
        elif userdata.guest_num_in == 2:
            return 'secound'
        else:
            pass


class SayFinish(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['success'])
        # Node
        self.node = node
        self.logger = node.get_logger()

    def execute(self, userdata):
        time.sleep(1.0)
        return 'success'


class Receptionist(Node):
    def __init__(self):
        super().__init__('receptionist')

    def execute(self):
        # Create a SMACH state machine
        sm_top = smach.StateMachine(outcomes=['end'])
        #sm_sub = smach.StateMachine(outcome=['End of guide'])
        sm_top.userdata.guest_num = 1
        sm_top.userdata.location_name = 'wait_position'
        # Open the container
        with sm_top:
            # Add states
            smach.StateMachine.add(
                    'ENTER_ROOM', EnterRoom(self),
                    transitions={'success':'NAVIGATION'})
            smach.StateMachine.add(
                    'NAVIGATION', Navigation(self),
                    transitions={'wait_position':'WAIT_GUEST',
                                 'party_room':'SAY_INFO'},
                    remapping={'location_name_in':'location_name'})
            smach.StateMachine.add(
                    'WAIT_GUEST', WaitGuest(self),
                    transitions={'success':'GET_INFO'})
            smach.StateMachine.add(
                    'GET_INFO', GetInfo(self),
                    transitions={'success':'NAVIGATION',
                                 'secound':'GET_FEATURE'},
                    remapping={'guest_num_in':'guest_num',
                               'location_name_out':'location_name'})
            smach.StateMachine.add(
                    'GET_FEATURE', GetFeature(self),
                    transitions={'success':'NAVIGATION'})
            smach.StateMachine.add(
                    'SAY_INFO', SayInfo(self),
                    transitions={'success':'GUIDE_TO_SEAT'},
                    remapping={'guest_num_in':'guest_num'})
            smach.StateMachine.add(
                    'GUIDE_TO_SEAT', GuideToSeat(self),
                    transitions={'success':'NAVIGATION',
                                 'secound':'SAY_FINISH'},
                    remapping={'guest_num_in':'guest_num',
                               'guest_num_out':'guest_num',
                               'location_name_out':'location_name'})
            smach.StateMachine.add(
                    'SAY_FINISH', SayFinish(self),
                    transitions={'success':'end'})
        
        # Viewer
        sis = smach_ros.IntrospectionServer('receptionist', sm_top, '/SM_ROOT')
        sis.start()

        # Execute
        outcome = sm_top.execute()
        self.get_logger().info(f"outcome: {outcome}")


def main():
    global speech

    rclpy.init()
    recp = Receptionist()
    recp.execute()
