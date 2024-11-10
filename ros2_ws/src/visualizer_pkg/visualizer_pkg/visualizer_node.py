import rclpy #type: ignore
from rclpy.node import Node #type: ignore
from std_msgs.msg import Float32 #type: ignore

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        self.temperature = None
        self.correction = None

        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.listener_callback,
            10)

        self.subscription = self.create_subscription(
            Float32,
            'correction',
            self.correction_callback,
            10)

    def listener_callback(self, msg):
        self.temperature = msg.data
        self.display_data() 
    
    def correction_callback(self, msg):
        self.correction = msg.data
        self.display_data()  

    def display_data(self):
        # Controlla che entrambi i valori siano disponibili
        if self.temperature is not None and self.correction is not None:
            self.get_logger().info(
                f'Temperatura: {self.temperature}, Correzione PID: {self.correction}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
