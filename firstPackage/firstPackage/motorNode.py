import time
import RPi.GPIO as GPIO
import rclpy
from std_msgs.msg import String, Int8
from rclpy.node import Node

# motor_EN_A: Pin7  |  motor_EN_B: Pin11
# motor_A:  Pin8,Pin10    |  motor_B: Pin13,Pin12

Motor_A_EN = 4
Motor_B_EN = 17

Motor_A_Pin1 = 26
Motor_A_Pin2 = 21
Motor_B_Pin1 = 27
Motor_B_Pin2 = 18

Dir_forward = 1
Dir_backward = 0

left_forward = 1
left_backward = 0

right_forward = 0
right_backward = 1

pwn_A = 0
pwm_B = 0


def setup():
    print('--> MOTORS node')
    rclpy.init()
    node = rclpy.create_node('motors_node')
    global pwm_A, pwm_B
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Motor_A_EN, GPIO.OUT)
    GPIO.setup(Motor_B_EN, GPIO.OUT)
    GPIO.setup(Motor_A_Pin1, GPIO.OUT)
    GPIO.setup(Motor_A_Pin2, GPIO.OUT)
    GPIO.setup(Motor_B_Pin1, GPIO.OUT)
    GPIO.setup(Motor_B_Pin2, GPIO.OUT)

    try:
        pwm_A = GPIO.PWM(Motor_A_EN, 1000)
        pwm_B = GPIO.PWM(Motor_B_EN, 1000)
    except:
        pass

    """def check_movement(movement):
        cases = {
            0: stop_motors,
            1: turn_left,
            2: move_motors,
            3: turn_right,
        }

        selected_function = cases.get(movement.data, stop_motors)
        selected_function()"""

    def check_movement(movement):
        move_arguments = {
            0: (95, 'no', 'straight'),
            #right e left invertiti perché la macchina ha i motori al contrario
            1: (100, 'forward', 'right'),
            2: (80, 'forward', 'straight'),
            3: (100, 'forward', 'left'),
        }

        args = move_arguments.get(movement.data, (0, "stop", 0))
        move(*args)

    def motor_left(status, direction, speed):  # Motor 2 positive and negative rotation
        if status == 0:  # stop
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
            GPIO.output(Motor_B_Pin2, GPIO.LOW)
            GPIO.output(Motor_B_EN, GPIO.LOW)
        else:
            if direction == Dir_backward:
                GPIO.output(Motor_B_Pin1, GPIO.HIGH)
                GPIO.output(Motor_B_Pin2, GPIO.LOW)
                pwm_B.start(10)
                pwm_B.ChangeDutyCycle(speed)
            elif direction == Dir_forward:
                GPIO.output(Motor_B_Pin1, GPIO.LOW)
                GPIO.output(Motor_B_Pin2, GPIO.HIGH)
                pwm_B.start(0)
                pwm_B.ChangeDutyCycle(speed)

    def motorStop():  # Motor stops
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.LOW)
        GPIO.output(Motor_B_EN, GPIO.LOW)

    def motor_right(status, direction, speed):  # Motor 1 positive and negative rotation
        if status == 0:  # stop
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
            GPIO.output(Motor_A_EN, GPIO.LOW)
        else:
            if direction == Dir_forward:  #
                GPIO.output(Motor_A_Pin1, GPIO.HIGH)
                GPIO.output(Motor_A_Pin2, GPIO.LOW)
                pwm_A.start(10)
                pwm_A.ChangeDutyCycle(speed)
            elif direction == Dir_backward:
                GPIO.output(Motor_A_Pin1, GPIO.LOW)
                GPIO.output(Motor_A_Pin2, GPIO.HIGH)
                pwm_A.start(0)
                pwm_A.ChangeDutyCycle(speed)
        return direction

    def move(speed, direction, turn, radius=0.6):  # 0 < radius <= 1
        # speed = 100
        if direction == 'forward':
            if turn == 'right':
                motor_left(0, left_backward, int(speed * radius))
                motor_right(1, right_forward, speed)
            elif turn == 'left':
                motor_left(1, left_forward, speed)
                motor_right(0, right_backward, int(speed * radius))
            else:
                motor_left(1, left_forward, speed)
                motor_right(1, right_forward, speed)
        elif direction == 'backward':
            if turn == 'right':
                motor_left(0, left_forward, int(speed * radius))
                motor_right(1, right_backward, speed)
            elif turn == 'left':
                motor_left(1, left_backward, speed)
                motor_right(0, right_forward, int(speed * radius))
            else:
                motor_left(1, left_backward, speed)
                motor_right(1, right_backward, speed)
        elif direction == 'no':
            if turn == 'right':
                motor_left(1, left_backward, speed)
                motor_right(1, right_forward, speed)
            elif turn == 'left':
                motor_left(1, left_forward, speed)
                motor_right(1, right_backward, speed)
            else:
                motorStop()
        else:
            pass

    def move_motors():
        print('Moving Straight')
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.HIGH)
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.HIGH)
        pwm_B.start(0)
        pwm_B.ChangeDutyCycle(100)
        pwm_A.start(80)
        pwm_A.ChangeDutyCycle(100)

    def stop_motors():
        print('Stopping Car')
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.LOW)
        GPIO.output(Motor_B_EN, GPIO.LOW)

    def turn_left():
        print('Turning left')
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.HIGH)
        pwm_B.start(80)

    def turn_right():  # Motor 1 positive and negative rotation
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.HIGH)
        pwm_A.start(80)

    node.create_subscription(Int8, 'motors', check_movement, 10)
    node.create_subscription(Int8, '/motorsProva', check_movement, 10)

    rclpy.spin(node)

    if KeyboardInterrupt:
        # node.destroy_timer(timer)
        node.destroy_node()
        rclpy.shutdown()


def destroy():
    # stop_motors()
    GPIO.cleanup()


if __name__ == '__main__':
    setup()
