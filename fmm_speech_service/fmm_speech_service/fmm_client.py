import rclpy
import random
from airobot_interfaces.srv import StringCommand

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('fmm_client')

    client = node.create_client(StringCommand, 'fmm_speech_service/wake_up')

    req = StringCommand.Request()
    for i in range(1):
    # 3人の名前と性別をサービスノードに要求するリクエストを送信
        person = random.choice([("male", "Robert"), ("male", "Leonardo"), ("female", "Elizabeth")])
        req.name = person[1]
        req.gender = person[0]

        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            response = future.result()
    # 追加：サービスノードから返された名前と性別をログに表示
            node.get_logger().info(f'{response.answer_people}')
        else:
            node.get_logger().info('サービスが応答しませんでした。')

        # if future.result() is not None:
        #     response = future.result()
        # # 追加：サービスノードから返された名前と性別をログに表示
        #     person = random.choice([("male", "Robert"), ("male", "Leonardo"), ("female", "Elizabeth")])
        #     node.get_logger().info(f'{person[1]} is {person[0]}')
        # else:

        #     node.get_logger().info('サービスが応答しませんでした。')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
