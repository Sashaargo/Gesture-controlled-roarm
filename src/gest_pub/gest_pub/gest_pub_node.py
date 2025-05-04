import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import mediapipe as mp
import math
import time
import json

class GesturePublisher(Node):
    def __init__(self):
        super().__init__('gesture_publisher')
        self.publisher_ = self.create_publisher(String, 'gesture_topic', 10)
        self.timer = self.create_timer(0.1, self.process_frame)

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.cap = cv2.VideoCapture(0)
        self.last_sent_time = 0
        self.send_interval = 0.1
        self.led_on = False

        # Параметры движения
        self.y_top = 60
        self.y_bottom = 460
        self.x_forward = 460
        self.x_backward = 60
        self.x_left = 0
        self.x_right = 640
        self.y_left_value = 250
        self.y_right_value = -250
        self.t_min = 1.3
        self.t_max = 3.2
        self.t_close_threshold = 0.2
        self.t_far_threshold = 0.5
        self.z_min = -120
        self.z_max = 400
        self.z_close =220
        self.z_far = 300

    def calc_distance(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def publish_json(self, data):
        msg = String()
        msg.data = json.dumps(data)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

    def process_frame(self):
        success, img = self.cap.read()
        if not success:
            return

        img = cv2.flip(img, 1)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)
        h, w, _ = img.shape

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            lm_list = [(int(lm.x * w), int(lm.y * h)) for lm in hand_landmarks.landmark]

            x_hand = lm_list[9][0]
            y_hand = lm_list[9][1]

            # X
            y_clamped = max(min(y_hand, self.y_bottom), self.y_top)
            x_mapped = int((1 - (y_clamped - self.y_top) / (self.y_bottom - self.y_top)) * (self.x_forward - self.x_backward) + self.x_backward)

            # Y
            x_clamped = max(min(x_hand, self.x_right), self.x_left)
            y_mapped = (1 - (x_clamped - self.x_left) / (self.x_right - self.x_left)) * (self.y_left_value - self.y_right_value) + self.y_right_value

            # Z
            dist_z = self.calc_distance(lm_list[0], lm_list[12])
            if dist_z <= self.z_close:
                z_mapped = self.z_max
            elif dist_z >= self.z_far:
                z_mapped = self.z_min
            else:
                ratio_z = (dist_z - self.z_close) / (self.z_far - self.z_close)
                z_mapped = self.z_max - (self.z_max - self.z_min) * ratio_z

            # T
            dist_t_raw = self.calc_distance(lm_list[8], lm_list[12])
            hand_scale = self.calc_distance(lm_list[0], lm_list[12]) + 1e-6
            dist_t = dist_t_raw / hand_scale

            if dist_t <= self.t_close_threshold:
                t_mapped = self.t_max
            elif dist_t >= self.t_far_threshold:
                t_mapped = self.t_min
            else:
                ratio_t = (dist_t - self.t_close_threshold) / (self.t_far_threshold - self.t_close_threshold)
                t_mapped = self.t_max - (self.t_max - self.t_min) * ratio_t

            # Основная команда
            if time.time() - self.last_sent_time > self.send_interval:
                data = {
                    "T": 1041,
                    "id": 1,
                    "x": x_mapped,
                    "y": round(y_mapped, 2),
                    "z": round(z_mapped, 2),
                    "t": round(t_mapped, 2)
                }
                self.publish_json(data)
                self.last_sent_time = time.time()

            # === Жест ✌️ (указательный и средний вверх, остальные вниз)
            index_up = lm_list[8][1] < lm_list[6][1]
            middle_up = lm_list[12][1] < lm_list[10][1]
            ring_up = lm_list[16][1] < lm_list[14][1]
            pinky_up = lm_list[20][1] < lm_list[18][1]

            is_peace_sign = index_up and middle_up and not ring_up and not pinky_up

            if is_peace_sign and not self.led_on:
                self.publish_json({"T": 114, "id": 1, "led": 255})
                self.get_logger().info("✌️ LED ON gesture detected")
                self.led_on = True

            elif not is_peace_sign and self.led_on:
                self.publish_json({"T": 115, "id": 1})
                self.get_logger().info("💡 LED OFF gesture detected")
                self.led_on = False

            # Отображение
            mp.solutions.drawing_utils.draw_landmarks(img, hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS)
            cv2.putText(img, f"X: {x_mapped}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(img, f"Y: {round(y_mapped,1)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
            cv2.putText(img, f"Z: {round(z_mapped,1)} (d: {int(dist_z)})", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100,200,255), 2)
            cv2.putText(img, f"T: {round(t_mapped,2)} (rel: {round(dist_t,2)})", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,100,100), 2)

        # Показываем изображение
        cv2.imshow("Manipulator Normalized Control", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = GesturePublisher()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
