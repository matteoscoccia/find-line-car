import RPi.GPIO as GPIO
import time
import rclpy
from std_msgs.msg import String
import random


line_pin_right = 19
line_pin_middle = 16
line_pin_left = 20


def setup():
    print('--> RANDOM MOVEMENT node')
    rclpy.init()
    node = rclpy.create_node('movement_node')
    mov_pub = node.create_publisher(String, 'random_movement', 10)

    def timer_callback():
        mov = randomMovement()
        print(mov)
        node.get_logger().info('Publishing movement: "%s"' % mov)
        mov_pub.publish(mov)


    timer_period = 1  # seconds
    timer = node.create_timer(timer_period, timer_callback)
    rclpy.spin(node)


def randomMovement():
    mov = String()
    mov.data = random.choice(["left", "right", "straight"])
    return mov


if __name__ == '__main__':
    setup()
    while 1:
        run()
