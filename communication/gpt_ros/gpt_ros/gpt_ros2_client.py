import rclpy
from rclpy.node import Node
from gpt_interface.srv import GPTText
from std_msgs.msg import String


class GPTClient(Node):

    def __init__(self):
        super().__init__('gpt_ros2_client')
        self.cli = self.create_client(GPTText, 'GPT_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GPTText.Request()
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('GPT Client is ready.')
        self.is_waiting_for_response = False
        self.publisher = self.create_publisher(String, 'commands_to_bridge', 10000)

    def timer_callback(self):
        if self.is_waiting_for_response is True:
            return

        user_prompt = input("You: ")
        self.req = GPTText.Request()
        self.req.request_text = user_prompt
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.execution_callback)
        self.is_waiting_for_response = True
        # self.get_logger().info('Sent request: %s' % self.req.request_text)
        self.timer.reset()

    def execution_callback(self, future):
        try:
            response = future.result()
            print("\n"+response.response_text+"\n")
            msg = String()
            msg.data = response.response_text
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % e)
        finally:
            self.is_waiting_for_response = False


def main(args=None):
    rclpy.init(args=args)
    gpt_client = GPTClient()
    rclpy.spin(gpt_client)
    gpt_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
