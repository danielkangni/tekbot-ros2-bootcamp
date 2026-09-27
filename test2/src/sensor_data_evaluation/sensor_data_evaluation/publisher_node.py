import json
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SensorDataPublisher(Node):

    def __init__(self):
        super().__init__('sensor_data_publisher')
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        data = {
            'temperature': round(random.uniform(15.0, 35.0), 2),
            'humidity': round(random.uniform(30.0, 70.0), 2),
            'pressure': round(random.uniform(950.0, 1050.0), 2),
        }
        msg = String()
        msg.data = json.dumps(data)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publié: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
