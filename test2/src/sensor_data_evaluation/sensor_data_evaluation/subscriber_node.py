import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TEMPERATURE_RANGE = (15.0, 35.0)
HUMIDITY_RANGE = (30.0, 70.0)
PRESSURE_RANGE = (950.0, 1050.0)


class SensorDataSubscriber(Node):

    def __init__(self):
        super().__init__('sensor_data_subscriber')
        self.subscription = self.create_subscription(
            String, 'sensor_data', self.listener_callback, 10
        )

    def listener_callback(self, msg):
        data = json.loads(msg.data)
        self._check('Température', data['temperature'], TEMPERATURE_RANGE, '°C')
        self._check('Humidité', data['humidity'], HUMIDITY_RANGE, '%')
        self._check('Pression', data['pressure'], PRESSURE_RANGE, 'hPa')

    def _check(self, label, value, value_range, unit):
        low, high = value_range
        if low <= value <= high:
            self.get_logger().info(f'{label} OK: {value}{unit}')
        else:
            self.get_logger().warn(f'{label} HORS PLAGE: {value}{unit} (attendu [{low}, {high}])')


def main(args=None):
    rclpy.init(args=args)
    node = SensorDataSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
