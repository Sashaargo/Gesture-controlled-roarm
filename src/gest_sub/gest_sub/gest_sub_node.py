import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import requests

class GestureSubscriber(Node):
    
    def __init__(self):
        super().__init__('gesture_subscriber')
        self.subscription = self.create_subscription(
            String,
            'gesture_topic',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.ip_addr = "192.168.150.47"

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            x = data.get("x", 0)
            y = data.get("y", 0.0)
            z = data.get("z", 0.0)
            t = data.get("t", 0.0)
            url = f"http://{self.ip_addr}/js?json={json.dumps(data)}"
            response = requests.get(url, timeout=1)
            self.get_logger().info(f"Sent: {data} → {response.status_code}")

            # Здесь можно добавить отправку на робот, лог, или другую логику
            self.get_logger().info(f"Received gesture data → x: {x}, y: {y}, z: {z}, t: {t}")
        
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON message")

def main(args=None):
    rclpy.init(args=args)
    node = GestureSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
