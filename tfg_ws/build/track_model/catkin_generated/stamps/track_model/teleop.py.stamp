#!/usr/bin/env python3

from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QPushButton, QGridLayout, QSlider, QCheckBox
from PyQt5.QtGui import QPixmap
from PyQt5 import QtGui
from PyQt5.QtCore import Qt

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

import rospy
from std_msgs.msg import Float64

import time, math
import topics

DEFAULT_SPEED = 10
DEFAULT_ANGULAR_SPEED = 5

RECT_ANGLE_TURN_WAIT = 0.8
LINEAR_WAIT = 1

LINEAR_CONSTANT = 5
ANGLE_MULTIPLYER = 15


bridge = CvBridge()

class CameraImage:
    def __init__(self, interface):
        self.topic = "/camera/image"
        self.sub = rospy.Subscriber(self.topic, Image, self.cb)
        self.window = interface
        self.label = interface.image

    def cb(self, msg):
        try:
        # Convert your ROS Image message to OpenCV2
            cvImg = bridge.imgmsg_to_cv2(msg, "bgr8")
            qtImg = convertCVtoQT(self.window, cvImg)
            self.label.setPixmap(qtImg)
        except CvBridgeError:
            print("ERROR")


def convertCVtoQT(window, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(int(window.display_width/4), int(window.display_height/4), Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)


class CarControl:
    def __init__(self):
        self.rightRearPub = rospy.Publisher(topics.RIGHT_REAR_TOPIC, Float64, queue_size=10)
        self.leftRearPub = rospy.Publisher(topics.LEFT_REAR_TOPIC, Float64, queue_size=10)
        self.rightFrontPub = rospy.Publisher(topics.RIGHT_FRONT_TOPIC, Float64, queue_size=10)
        self.leftFrontPub = rospy.Publisher(topics.LEFT_FRONT_TOPIC, Float64, queue_size=10)

        self.leftSteeringPub = rospy.Publisher(topics.LEFT_DIRECTION_TOPIC, Float64, queue_size=10)
        self.rightSteeringPub = rospy.Publisher(topics.RIGHT_DIRECTION_TOPIC, Float64, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

    def linearDrive(self, v):
        linearMsg = Float64()
        linearMsg.data = float(v)
        self.rightRearPub.publish(linearMsg)
        self.leftRearPub.publish(linearMsg) 
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
        time.sleep(0.5)
        
        linearSpeed = abs(w * LINEAR_CONSTANT)
        self.linearDrive(linearSpeed)
        
    def drive(self, v, w):
        self.linearDrive(v)
        self.angularDrive(w)
        self.rate.sleep()

def fwdFunct():
    wheelControl.drive(DEFAULT_SPEED,0)
    time.sleep(LINEAR_WAIT)
    wheelControl.drive(0,0)
        
def bwdFunct():
    wheelControl.drive(-DEFAULT_SPEED,0)
    time.sleep(LINEAR_WAIT)
    wheelControl.drive(0,0)
    
def leftFunct():
    wheelControl.drive(0,DEFAULT_ANGULAR_SPEED)
    time.sleep(RECT_ANGLE_TURN_WAIT)
    wheelControl.drive(0,0)

def rightFunct():
    wheelControl.drive(0, -DEFAULT_ANGULAR_SPEED)
    time.sleep(RECT_ANGLE_TURN_WAIT)
    wheelControl.drive(0,0)
 
def stopFunct():
    wheelControl.drive(0,0)

def ccFunct(butt):
    if not butt.isChecked():
        wheelControl.drive(0,0)
    else:
        wheelControl.drive(DEFAULT_SPEED,0)

class InterfaceWindow():
    def __init__(self):
        self.app = QApplication([])
        self.window = QWidget()
        self.layout = QGridLayout()
        self.window.setWindowTitle("Teleop Interface")
        self.window.display_width = 640
        self.window.display_height = 480

    def addButtons(self):
        fwdButton = QPushButton("Forward")
        fwdButton.clicked.connect(fwdFunct)
        self.layout.addWidget(fwdButton, 0, 1)

        bwdButton = QPushButton("Backwards")
        bwdButton.clicked.connect(bwdFunct)
        self.layout.addWidget(bwdButton, 2, 1)

        stopButton = QPushButton("Stop") 
        stopButton.clicked.connect(stopFunct)
        self.layout.addWidget(stopButton, 1, 1)

        leftButton = QPushButton("Left") 
        leftButton.clicked.connect(leftFunct)
        self.layout.addWidget(leftButton, 1, 0)

        rightButton = QPushButton("Right") 
        rightButton.clicked.connect(rightFunct)
        self.layout.addWidget(rightButton, 1, 2)

        ccCheckBox = QCheckBox("Cruise Control")
        ccCheckBox.setChecked(False)
        ccCheckBox.stateChanged.connect(lambda:ccFunct(ccCheckBox))
        self.layout.addWidget(ccCheckBox, 1, 3)

    def treatValue(self):
        wheelControl.inputSteeringAngle(self.directionalSlider.value())

    def addSliders(self):
        self.directionalSlider = QSlider(Qt.Horizontal)
        self.directionalSlider.setMinimum(-45)
        self.directionalSlider.setMaximum(45)
        self.directionalSlider.setValue(0)
        self.directionalSlider.setTickPosition(QSlider.TicksBelow)
        self.directionalSlider.setTickInterval(1)
        self.layout.addWidget(self.directionalSlider, 2, 3)
        self.directionalSlider.valueChanged.connect(self.treatValue)

    def addImages(self):
        self.window.image = QLabel("Camera View")
        self.layout.addWidget(self.window.image, 3, 1)

    def executeInterface(self):
        self.window.setLayout(self.layout)
        self.window.show()
        self.app.exec_()


rospy.init_node('interface_node')

wheelControl = CarControl()

interface = InterfaceWindow()
interface.addButtons()
interface.addImages()
interface.addSliders()

image = CameraImage(interface.window)

def main():
    interface.executeInterface()
    rospy.spin()

if __name__ == '__main__':
    main()