import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')
        print('--> CAMERA node')
        self.color_publisher = self.create_publisher(String, 'color_detected', 10)
        self.timer = self.create_timer(1.0 / 30, self.capture_and_publish)
        self.image_publisher = self.create_publisher(CompressedImage, '/car_1/camera', 10)
        self.bridge = CvBridge()

    def capture_and_publish(self):
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Adjust the camera index as needed
        ret, frame = cap.read()
        #self.get_logger().info('Camera %s' % cap.isOpened())
        #self.get_logger().info('Read %s' % ret)
        # cv2.imwrite("PROVAROBA.png",frame)
        # print(cv2.getBuildInformation())

        # Set the desired width and height for the camera resolution
        desired_width = 640
        desired_height = 480

        # Set the camera resolution
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)

        if ret:
            # Check if red or green color is present
            red_detected = bool(self.is_red_present(frame))
            yellow_detected = bool(self.is_yellow_present(frame))
            purple_detected = bool(self.is_purple_present(frame))
            self.get_logger().info('Red? %s' % red_detected)
            self.get_logger().info('Yellow? %s' % yellow_detected)
            self.get_logger().info('Purple? %s' % purple_detected)
            color_detection_msg = String()

            if red_detected:
                color_detection_msg.data = "red"
            elif yellow_detected:
                color_detection_msg.data = "yellow"
            elif purple_detected :
                color_detection_msg.data = "purple"
            else :
                color_detection_msg.data = ""

            self.color_publisher.publish(color_detection_msg)

            #send image to rosbridge server
            _, compressed_data = cv2.imencode('.jpg', frame)
            ros_compressed_image_msg = CompressedImage()
            ros_compressed_image_msg.format = 'jpeg'
            ros_compressed_image_msg.data = compressed_data.tobytes()
            self.image_publisher.publish(ros_compressed_image_msg)

        else:
            self.get_logger().warning("Failed to capture an image.")

        cap.release()

    def is_red_present(self, image):
        # Check if red color is present
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, lower_red, upper_red)
        result = cv2.bitwise_and(image, image, mask=mask)

        return np.any(result)

    def is_yellow_present(self, image):
        # Check if blue color is present
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        result = cv2.bitwise_and(image, image, mask=mask)

        return np.any(result)

    def is_purple_present(self, image):
        lower_purple = np.array([125, 50, 50])
        upper_purple = np.array([155, 255, 255])

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, lower_purple, upper_purple)
        result = cv2.bitwise_and(image, image, mask=mask)

        return np.any(result)


def main(args=None):
    rclpy.init(args=args)
    color_detection_node = ColorDetectionNode()
    rclpy.spin(color_detection_node)
    color_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
