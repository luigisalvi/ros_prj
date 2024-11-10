import rclpy #type: ignore
from rclpy.node import Node #type: ignore
from std_msgs.msg import Float32 #type: ignore

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        self.subscription = self.create_subscription(
            Float32,
            'temperatura',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'Temperatura corrente: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
