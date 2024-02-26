import RPi.GPIO as GPIO
import time
import rclpy
from geometry_msgs.msg import Vector3

line_pin_right = 19
line_pin_middle = 16
line_pin_left = 20


def setup():
    print('--> FINDLINE node')
    rclpy.init()
    node = rclpy.create_node('findline_sensor')
    line_pub = node.create_publisher(Vector3, 'linedata', 10)
    #turn_data = Vector3()
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(line_pin_right, GPIO.IN)
    GPIO.setup(line_pin_middle, GPIO.IN)
    GPIO.setup(line_pin_left, GPIO.IN)

    def timer_callback():  # definition of a timer function that manages all the publications
        pins = checkpin()
        print(pins)
        node.get_logger().info('Publishing: "%s"' % pins)
        line_pub.publish(pins)


    timer_period = 0.05  # seconds
    timer = node.create_timer(timer_period, timer_callback)
    rclpy.spin(node)


def checkpin():
    status_right = GPIO.input(line_pin_right)
    status_middle = GPIO.input(line_pin_middle)
    status_left = GPIO.input(line_pin_left)
    print('LF3: %d   LF2: %d   LF1: %d\n' % (status_right, status_middle, status_left))
    pins = Vector3()
    pins.x = float(status_right)
    pins.y = float(status_middle)
    pins.z = float(status_left)
    return pins


if __name__ == '__main__':
    setup()
    while 1:
        run()
