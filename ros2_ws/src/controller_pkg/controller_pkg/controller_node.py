import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import Float32 # type: ignore

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32, 'correction', 10)
        #self.subscription

        self.setpoint_subscription = self.create_subscription(
            Float32,
            'setpoint',
            self.setpoint_callback,
            10)
        
        self.setpoint = 50.0  # Temperaruta di setpoint
        self.kp = 0.1  #P
        self.ki = 0.01 #I
        self.kd = 0.05 #D
        self.previous_error = 0.0
        self.integral = 0.0

    def setpoint_callback(self, msg):
        self.setpoint = msg.data
        self.get_logger().info(f'Setpoint updated to: {self.setpoint}')
        
    def listener_callback(self, msg):
        error = self.setpoint - msg.data
        self.integral += error
        derivative = error - self.previous_error
        control_signal = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        #self.get_logger().info(f'Controllo PID: {control_signal}')

        # Publish the control signal as a corrective action
        correction_msg = Float32()
        correction_msg.data = control_signal
        self.publisher_.publish(correction_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
