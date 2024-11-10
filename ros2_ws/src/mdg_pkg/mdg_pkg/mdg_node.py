import rclpy # type: ignore
from rclpy.node import Node #type: ignore
import random
from std_msgs.msg import Float32 # type: ignore

class mdgNode(Node):
    def __init__(self):
        super().__init__('mdg_node')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        self.subscription = self.create_subscription(
            Float32,
            'correction',
            self.correction_callback,
            10)
        self.timer = self.create_timer(3.0, self.publish_temperature)
        self.temperature = 40.0  # Temperatura iniziale

    def publish_temperature(self):
        # Rumore sulla temperatura iniziale ad ogni passo
        self.temperature += random.uniform(-1, 3)
        msg = Float32()
        msg.data = self.temperature
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Pubblicata temperatura: {msg.data}')
    
    def correction_callback(self, msg):
        # Adjust the temperature based on the corrective action
        self.temperature += msg.data
        #self.get_logger().info(f'Correzione ricevuta: {msg.data}, Nuova temperatura: {self.temperature}')

def main(args=None):
    rclpy.init(args=args)
    node = mdgNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
