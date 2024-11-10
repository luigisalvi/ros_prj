import rclpy # type: ignore
from rclpy.node import Node #type: ignore
import random
from std_msgs.msg import Float32 # type: ignore

class mdgNode(Node):
    def __init__(self):
        super().__init__('mdg_node')
        self.publisher_ = self.create_publisher(Float32, 'temperatura', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)
        self.temperature = 20.0  # Temperatura iniziale

    def publish_temperature(self):
        # Rumore sulla temperatura iniziale ad ogni passo
        self.temperature += random.uniform(-0.8, 0.8)
        msg = Float32()
        msg.data = self.temperature
        self.publisher_.publish(msg)
        self.get_logger().info(f'Pubblicata temperatura: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = mdgNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
