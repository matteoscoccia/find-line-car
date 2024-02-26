import time
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Range
from std_msgs.msg import Int8
from geometry_msgs.msg import Vector3

class ControllerNode(Node):
    red_traffic_light = False
    last_color = ""
    block = False  

    def __init__(self):
        super().__init__('controller_node')
        print('--> CONTROLLER node')
        
        self.motors_pub = self.create_publisher(Int8, 'motors', 10)
        self.block_pub = self.create_publisher(String, '/block', 10)

        self.create_subscription(Vector3, 'linedata', self.status_output, 10)
        self.create_subscription(String, 'random_movement', self.random_movement, 10)
        self.create_subscription(String, 'color_detected', self.color_detection, 10)
        self.create_subscription(String, '/block', self.block_detection, 10)

    def status_output(self, linedata):
        mov = Int8()
        self.get_logger().info('Red?: %s' % ControllerNode.red_traffic_light)
        self.get_logger().info('Block?: %s' % ControllerNode.block)

        if not ControllerNode.red_traffic_light and not ControllerNode.block:
            if linedata.x == 1.0 and linedata.z == 0.0:
                # turn left
                self.get_logger().info('TURN LEFT')
                mov.data = 1
            elif linedata.z == 1.0 and linedata.x == 0.0:
                # turn right
                self.get_logger().info('TURN RIGHT')
                mov.data = 3
            elif linedata.x == 0.0 and linedata.y == 1.0 and linedata.z == 0.0:
                # go ahead
                self.get_logger().info('GO AHEAD')
                mov.data = 2
            else:
                # stop
                self.get_logger().info('STOP')
                mov.data = 0
        else :
            # stop
            self.get_logger().info('STOP')
            mov.data = 0

        self.motors_pub.publish(mov)

    def random_movement(self, movement):
        mov = Int8()
        self.get_logger().info('Red?: %s' % ControllerNode.red_traffic_light)
        self.get_logger().info('Block?: %s' % ControllerNode.block)

        if not ControllerNode.red_traffic_light and not ControllerNode.block:
            if movement.data == "left":
                # turn left
                self.get_logger().info('TURN LEFT')
                mov.data = 1
            elif movement.data == "right":
                # turn right
                self.get_logger().info('TURN RIGHT')
                mov.data = 3
            elif movement.data == "straight":
                # go ahead
                self.get_logger().info('GO AHEAD')
                mov.data = 2
            else:
                # stop
                self.get_logger().info('STOP')
                mov.data = 0
        else :
            # stop
            self.get_logger().info('STOP')
            mov.data = 0

        self.motors_pub.publish(mov)

    def color_detection(self, data):
        if not ControllerNode.block :
            if data.data == "red":
                ControllerNode.red_traffic_light = True
                #if ControllerNode.last_color == "yellow" :
                #    self.send_block(False)
                #ControllerNode.last_color = "red"
            elif data.data == "yellow" :
                self.send_block(True)
                ControllerNode.red_traffic_light = False
            elif data.data == "purple":
                ControllerNode.red_traffic_light = False
        elif data.data == "purple":
                ControllerNode.block = False
                ControllerNode.red_traffic_light = False

    def send_block (self, boolean):
        app1 = self.get_namespace().replace('/', "")
        app2 = str(boolean)
        combined = f"{app1}-{app2}"
        app = String()
        app.data = combined
        self.block_pub.publish(app)

    def block_detection(self, data):
        components = data.data.split("-")
        namespace = components[0]
        value = True if components[1] == "True" else False
        if not (self.get_namespace().replace('/', "") == namespace) :
            ControllerNode.block = value

    def shutdown(self):
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
