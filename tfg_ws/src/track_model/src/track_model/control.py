from cmath import pi
from track_model import topics

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg      import Float64

from time import sleep
import math

LINEAR_CONSTANT = 5
ANGLE_MULTIPLYER = 15
MAX_WHEEL_ANGLE = math.pi/4

class CarControl:
    def __init__(self):
        self.rightRearPub = rospy.Publisher(topics.RIGHT_REAR_TOPIC, Float64, queue_size=10)
        self.leftRearPub = rospy.Publisher(topics.LEFT_REAR_TOPIC, Float64, queue_size=10)
        self.rightFrontPub = rospy.Publisher(topics.RIGHT_FRONT_TOPIC, Float64, queue_size=10)
        self.leftFrontPub = rospy.Publisher(topics.LEFT_FRONT_TOPIC, Float64, queue_size=10)

        self.leftSteeringPub = rospy.Publisher(topics.LEFT_DIRECTION_TOPIC, Float64, queue_size=10)
        self.rightSteeringPub = rospy.Publisher(topics.RIGHT_DIRECTION_TOPIC, Float64, queue_size=10)
        
        self.dataPub = rospy.Publisher(topics.COMMANDED_DATA_TOPIC, Twist, queue_size=10)
        
        self.rate = rospy.Rate(10) # 10hz

    def linearDrive(self, v):
        linearMsg = Float64()
        linearMsg.data = float(v)
       
        self.leftFrontPub.publish(linearMsg)
        self.rightFrontPub.publish(linearMsg)
        
    def inputSteeringAngle(self, angle):
        rads = -1 * math.radians(angle)
        msg = Float64()
        msg.data = float(rads)
        self.leftSteeringPub.publish(msg)
        self.rightSteeringPub.publish(msg)
        
    def angularDrive(self, w):
        angle = w * ANGLE_MULTIPLYER
        self.inputSteeringAngle(angle)        
        linearSpeed = abs(w * LINEAR_CONSTANT)
        self.linearDrive(linearSpeed)
        
    def drive(self, v, w):
        self.linearDrive(v)
        self.angularDrive(w)
        self.rate.sleep()

    def commandVel(self, v, w):
        msg = Twist()
        
        msg.linear.x = v
        msg.linear.y = 0
        msg.linear.z = 0
        
        if abs(w) > MAX_WHEEL_ANGLE:
            if w < 0:
                w = -MAX_WHEEL_ANGLE
            else:
                w = MAX_WHEEL_ANGLE
        
        
        msg.angular.x = w
        msg.angular.y = 0
        msg.angular.z = 0
        
        self.dataPub.publish(msg)
        
    
        