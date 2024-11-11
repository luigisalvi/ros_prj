import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Float32  # type: ignore

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')
        self.publisher_ = self.create_publisher(Float32, 'setpoint', 10)
        
        self.timer = self.create_timer(2.0, self.publish_setpoint)
        
    def publish_setpoint(self):
        try:
            setpoint_value = float(input("Enter the setpoint value: "))
            msg = Float32()
            msg.data = setpoint_value
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published setpoint: {setpoint_value}")
        except ValueError:
            self.get_logger().error("Invalid input. Please enter a numeric value.")

def main(args=None):
    rclpy.init(args=args)
    node = InputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
