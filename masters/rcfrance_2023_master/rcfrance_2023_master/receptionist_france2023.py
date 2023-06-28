import math
import pyttsx3
import time
import rclpy
from subprocess import call
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
import smach
import smach_ros
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# Module
from happymini_manipulation.motor_controller import JointController
from happymini_teleop.base_control import BaseControl
# Custom msg
from happymini_msgs.action import GraspBag
from happymini_msgs.srv import NaviLocation, TextToSpeech, SpeechToText, WavPlay, NameDetect, DetectEmptyChair, AttributeRecognition, DetectPerson, TrackPerson


name_list = []
fav_drink_list =[]
feature_list = []
is_second = False

def synthesis2(text = None):
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


class AttributeRecog(Node):
    def __init__(self):
        super().__init__("attrib")
        self.node = rclpy.create_node('attrib_recog')
        self.__client = self.node.create_client(AttributeRecognition, '/fmm_attrib_service/recognize')
        self.__req = None
        self.future = None

    def execute(self, img : Image, env_img : Image) -> str:
        self.__req = AttributeRecognition.Request()
        self.__req.input = img
        self.__req.environment_image = env_img
        self.future = self.__client.call_async(self.__req)
        rclpy.spin_until_future_complete(self.node, self.future)

        while not self.future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.future.result() is not None:
            response = self.future.result()
        else:
           self.get_logger().info('サービスが応答しませんでした。')
        return response.result


class TrackPersonClient(Node):
    def __init__(self):
        super().__init__("track_person") 
        self.__client = self.create_client(TrackPerson, 'recp/track_person')
        self.__req = None
        self.future = None

    def execute(self):
        self.__req = TrackPerson.Request()
        self.__req.command = ''
        self.future = self.__client.call_async(self.__req)
        rclpy.spin_until_future_complete(self, self.future)

        while not self.future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.future.result() is not None:
            response = self.future.result()
        else:
           self.get_logger().info('サービスが応答しませんでした。')
        return response.angle


class EnterRoom(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['success'])
        # Node
        self.node = node
        self.logger = node.get_logger()

    def execute(self, userdata):
        _ = self.node.tts("Start Receptionist")
        #synthesis2('Start Receptionist')
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
        self.navi = node.create_client(NaviLocation, 'navi_location_server')
        while not self.navi.wait_for_service(timeout_sec=1.0) and rclpy.ok():
            self.logger.info("/navi_location_server is not here ...")
        self.req = NaviLocation.Request()

    def do_navigation(self):
        # Call
        future = self.navi.call_async(self.req)
        # Waiting
        print(f'Start do_navigation')
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
        navi_result = False
        counter = 1
        while not navi_result and rclpy.ok():
            if counter > 3:
                break
            navi_result = self.do_navigation()
            counter += 1
        return userdata.location_name_in


class WaitGuest(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['success'],
                output_keys=['guest_num_out'])
        # Node
        self.node = node
        self.logger = node.get_logger()
        self.num_persoon = 0

    def execute(self, userdata):
        self.num_persoon += 1
        userdata.guest_num_out = self.num_persoon
        self.node.tts('Please come to front of me.')
        time.sleep(1.0)
        return 'success'


class GetInfo(smach.State):
    def __init__(self, node):
        smach.State.__init__(
                self,
                outcomes=['success'],
                input_keys=['guest_num_in'],
                output_keys=['guest_num_in', 'location_name_out', 'name_out', 'drink_out', 'feature_out'])
        # Node
        self.node = node
        self.logger = node.get_logger()
        # Service
        self.stt_srv = self.node.create_client(SpeechToText, 'stt')
        self.nd_srv = self.node.create_client(NameDetect, 'nd')
        while not self.stt_srv.wait_for_service(timeout_sec=0.5) and self.nd_srv.wait_for_service(timeout_sec=0.5) and rclpy.ok():
            self.logger.info("/sst and /nd are not here ...")
        self.stt_srv_req = SpeechToText.Request()
        self.nd_srv_req = NameDetect.Request()
        # AttributeRecog
        #self.__ar_client = self.node.create_client(AttributeRecognition, 'recp_find_seat')
        #self.__ar_req = None
        #self.ar_future = None
        # PersonDetector
        self.__pd_client = self.node.create_client(DetectPerson, '/fmm_person_service/detect')
        self.__pd_req = None
        self.pd_future = None
        self.__bridge = CvBridge()
        self.__cnt = 0
        # Params
        self.drinks = {'amelia':'coffee', 'charlie':'wine', 'hunter':'beer', 'max':'jintoonic'}

    def stt_send_request(self, cmd='start'):
        stt_srv_result = 'None'
        self.stt_srv_req.cmd = cmd

        stt_srv_future = self.stt_srv.call_async(self.stt_srv_req)
        while not stt_srv_future.done() and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if stt_srv_future.result() is not None:
            stt_srv_result = stt_srv_future.result().result
            print(f"sst: {stt_srv_result}")
            return stt_srv_result
        else:
            self.get_logger().info(f"Service call failed")
            return None

    def nd_send_request(self, text=None):
        nd_srv_result = 'None'
        self.nd_srv_req.text = text

        nd_srv_future = self.nd_srv.call_async(self.nd_srv_req)
        while not nd_srv_future.done() and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if nd_srv_future.result() is not None:
            nd_srv_result = nd_srv_future.result().result
            print(f"nd: {nd_srv_result}")
            return nd_srv_result
        else:
            self.get_logger().info("Service call failed")
            return None

    def execute(self, userdata):
        name_flg = False
        global name_list
        global fav_drink_list
        global feature_list
        while not name_flg:
            #synthesis2('What is your name?')
            _ = self.node.tts('What is your name?')
            _ = self.node.tts('"Say your name after the beep sound."')
            # Name detect
            name = self.stt_send_request()
            name = name.replace(" ","").replace(".","")
            name_d = self.nd_send_request(name)
            # Get Drink
            try:
                drink = self.drinks[name_d]
                name_flg = True
            except KeyError:
                _ = self.node.tts('Sorry. I did not catch that. Plrease try again.')
                #synthesis2('Sorry. I did not catch that. Plrease try again.')
                continue
            print(drink)
            name_list.append(name_d)
            fav_drink_list.append(drink)
        _ = self.node.tts('Hi, ' + name_d)
        # PersonDetector
        self.__pd_req = DetectPerson.Request()
        self.pd_future = self.__pd_client.call_async(self.__pd_req)
        rclpy.spin_until_future_complete(self.node, self.pd_future)
        print('Start Person Detector')
        img_response=None
        while not self.pd_future.done() and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.pd_future.result() is not None:
            img_response = self.pd_future.result()
            #img_cv = self.__bridge.imgmsg_to_cv2(response.result)
            #cv2.imwrite("~/main_ws/src/happymini_ros/masters/rcj_2023_master/img{0}.png".format(self.__cnt), img_cv)
            #self.__cnt += 1
            print(img_response.result)
            print(img_response.environment_image)
        else:
           self.get_logger().info('サービスが応答しませんでした。')
        # AttributeRecog
        print("Start attribute_recog")
        attribute_sentence = ar_node.execute(img_response.result, img_response.environment_image) 
        print(attribute_sentence)
        feature_list.append(attribute_sentence)
        #print("Start AttributeRecognition")
        #self.__ar_req = AttributeRecognition.Request()
        #self.__ar_req.input = img_response.result
        #self.__ar_req.environment_image = img_response.environment_image
        #print(self.__ar_req.input)
        #print(self.__ar_req.environment_image)
        #self.ar_future = self.__ar_client.call_async(self.__ar_req)
        #rclpy.spin_until_future_complete(self.node, self.ar_future)

        #while not self.ar_future.done() and rclpy.ok():
        #    rclpy.spin_once(self.node, timeout_sec=0.1)
        #if self.ar_future.result() is not None:
        #    response = self.ar_future.result()
        #else:
        #   self.get_logger().info('サービスが応答しませんでした。')
        userdata.name_out = name_d
        userdata.drink_out = drink
        userdata.feature_out = attribute_sentence
        #userdata.feature_out = response.result
        #print(userdata.name_out)
        #print(userdata.drink_out)
        #print(userdata.feature_out)
        _ = self.node.tts('Please follow me.')
        userdata.location_name_out = 'party_room'
        #print(f'{userdata.gust_num_in}')
        return 'success'


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
                input_keys=['guest_num_in',
                            'name_in',
                            'drink_in',
                            'feature_in'])
        # Node
        self.node = node
        self.logger = node.get_logger()

    def execute(self, userdata):
        global is_second
        global name_list
        global feature_list
        _ = self.node.tts('Name is ' + userdata.name_in)
        #synthesis2('Name is ' + userdata.name_in)
        time.sleep(1.0)
        _ = self.node.tts(userdata.name_in + "'s favorite drink is " + userdata.drink_in)
        #synthesis2('Drink is ' + userdata.drink_in)
        time.sleep(1.0)
        if is_second:
            first_sentence = f"Hey {name_list[0]}, I want to introduce {name_list[1]}."
            _ = self.node.tts(first_sentence)
            _ = self.node.tts(feature_list[1])
        #synthesis2(userdata.feature_in)
        is_second = True
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
        # Publisher
        self.twist_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        # Service
        self.__client = self.node.create_client(DetectEmptyChair, 'recp/find_seat')
        self.__req = None
        self.future = None
        # Module
        self.bc_node = BaseControl()
        # Value
        self.twist = Twist()

    def execute(self, userdata):
        self.__req = DetectEmptyChair.Request()
        self.__req.command = ""
        self.future = self.__client.call_async(self.__req)
        rclpy.spin_until_future_complete(self.node, self.future)
        while not self.future.done() and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.future.result() is not None:
            result = self.future.result()
        else:
            self.get_logger().info('サービスが応答しませんでした。')
        # 向く
        if len(result.angles) > 0:
            self.bc_node.rotate_angle(-1*result.angles[0])
        # しゃべる
        _ = self.node.tts('Please sit in the chair in the direction I am facing now. Please wait until I turn around.')
        #synthesis2('Please sit in the chair in the direction I am facing.')
        # tracking
        self.bc_node.rotate_angle(180, precision=0, speed=0.5, time_out=10)
        _ = self.node.tts("Please sit in the chair.")
        time.sleep(1.0)
        start_time = time.time()

        move_angle = 0
        cnt = 0
        
        while rclpy.ok():
            
            if time.time() - start_time > 20:
                self.twist.angular.z = 0.0
                self.twist_pub.publish(self.twist)
                break
            print("updating angle")

            import subprocess
            returnval = subprocess.run("ros2 service call /recp/track_person happymini_msgs/srv/TrackPerson command:\'\'", shell=True, executable="/bin/bash", capture_output=True, text=True).stdout
            stdoutlines = returnval.split("\n")
            angle = 0.0
            for i in range(len(stdoutlines)):
                if stdoutlines[i].find("angle=") != -1:
                    angle = float(stdoutlines[i][stdoutlines[i].find("angle=")+6:stdoutlines[i].find(")")])
                    print(angle)
                    break
            #sysres = call(["ros2", "topic", "list"], shell=True, executable="/bin/bash")
            move_angle = -1 * angle#-1 * tp_node.execute()
            self.twist.angular.z = math.radians(move_angle)*0.5
            if abs(move_angle) <= 5:
                self.twist.angular.z = 0.0
            if abs(self.twist.angular.z) > 0.7:
                self.twist.angular.z = (self.twist.angular.z/abs(self.twist.angular.z))*0.6
            self.twist_pub.publish(self.twist)
            print(move_angle, self.twist.angular.z)
            time.sleep(0.01)
            cnt += 1
            #self.bc_node.rotate_angle(move_angle, precision=0, speed=0.4, time_out=3)
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
        _ = self.node.tts('Finish Receptionist. Thank you very much.')
        return 'success'


class Receptionist(Node):
    def __init__(self):
        super().__init__('receptionist')
        self.tts_srv = self.create_client(TextToSpeech, 'mimic3_play_server')
        while not self.tts_srv and rclpy.ok():
            self.get_logger().info("/mimic3_play_server is not here ...")
        self.tts_req = TextToSpeech.Request()

    def tts(self, text):
        # Msg
        self.tts_req.text = text
        # Call
        future = self.tts_srv.call_async(self.tts_req)
        while not future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if future.result() is not None:
            result = future.result().result
            return result
        else:
            self.get_logger().info("Service call failed")
            return False

    def execute(self):
        # Create a SMACH state machine
        sm_top = smach.StateMachine(outcomes=['end'])
        #sm_sub = smach.StateMachine(outcome=['End of guide'])
        sm_top.userdata.guest_num = 1
        sm_top.userdata.location_name = 'wait_position'
        sm_top.userdata.name = None
        sm_top.userdata.drink = None
        sm_top.userdata.feature = None
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
                    transitions={'success':'NAVIGATION'},
                    remapping={'guest_num_in':'guest_num_out',
                               'location_name_out':'location_name',
                               'name_out':'name',
                               'drink_out':'drink',
                               'feature_out':'feature'})
            smach.StateMachine.add(
                    'SAY_INFO', SayInfo(self),
                    transitions={'success':'GUIDE_TO_SEAT'},
                    remapping={'guest_num_in':'guest_num',
                               'name_in':'name',
                               'drink_in':'drink',
                               'feature_in':'feature'})
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
        #sis = smach_ros.IntrospectionServer('receptionist', sm_top, '/SM_ROOT')
        #sis.start()

        # Execute
        outcome = sm_top.execute()
        self.get_logger().info(f"outcome: {outcome}")


def main():
    global ar_node, tp_node

    rclpy.init()
    ar_node = AttributeRecog()
    tp_node = TrackPersonClient()
    recp = Receptionist()
    recp.execute()
