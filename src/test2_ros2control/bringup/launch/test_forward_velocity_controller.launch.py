import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.velocity_sequence = [0.5, 0.8, 1.0, 0.2]  # Ejemplo de secuencia de velocidades
        self.sequence_index = 0
        self.timer = self.create_timer(10, self.publish_next_velocity)

    def publish_next_velocity(self):
        if self.sequence_index < len(self.velocity_sequence):
            msg = Float64MultiArray()
            msg.data = [self.velocity_sequence[self.sequence_index]]
            self.publisher_.publish(msg)
            self.get_logger().info('Publicando velocidad: %s' % str(self.velocity_sequence[self.sequence_index]))
            self.sequence_index += 1
        else:
            self.get_logger().info('Secuencia de velocidades completada.')

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
