import rclpy
from airobot_interfaces.srv import StringCommand

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('fmm_client')

    client = node.create_client(StringCommand, 'fmm_speech_service/wake_up')

    req = StringCommand.Request()
    req.command = 'Hello'

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('サービスが利用可能になるまで待機中...')

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        response = future.result()
        node.get_logger().info('応答_性別: ' + response.answer_gender)
        node.get_logger().info('応答_名前: ' + response.answer_name)
    else:
        node.get_logger().info('サービスが応答しませんでした。')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
