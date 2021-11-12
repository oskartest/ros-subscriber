#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from RPi import GPIO
from gpiozero import Motor

motor = Motor(forward=20, backward=21)
pin = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)

p = GPIO.PWM(pin, 50)


p.start(0)

SERVO_MAX_DUTY = 12.5
SERVO_MIN_DUTY = 2.5

steer = 0

sub = None

def setServoPos(degree):

    if degree > 180:
        degree = 180

    duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
    p.ChangeDutyCycle(duty)

def callback(data):
    steer = data.data
    rospy.loginfo(steer)
    angle = 90-steer*180
    setServoPos(angle)
    motor.forward(speed=0.4)
def main():
    global sub
    rospy.init_node('sub_steer', anonymous=True)
    sub = rospy.Subscriber('/steers', Float32, callback)
    rospy.spin()

    

if __name__ == '__main__':
    main()
