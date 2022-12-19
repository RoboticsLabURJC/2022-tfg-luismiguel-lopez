#!/usr/bin/env python3

from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QPushButton, QGridLayout, QSlider, QCheckBox
from PyQt5.QtGui import QPixmap
from PyQt5 import QtGui
from PyQt5.QtCore import Qt

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

import rospy

import time
import math
from track_model import control, topics

DEFAULT_SPEED = 10
DEFAULT_ANGULAR_SPEED = 5

RECT_ANGLE_TURN_WAIT = 0.8
LINEAR_WAIT = 1

bridge = CvBridge()

class CameraImage:
    def __init__(self, interface):
        self.topic = topics.CAMERA_TOPIC
        self.sub = rospy.Subscriber(self.topic, Image, self.cb)
        self.window = interface
        self.label = interface.image
        self.lastImageT = time.time() 

    def cb(self, msg):
        try:
        # Convert your ROS Image message to OpenCV2
            cvImg = bridge.imgmsg_to_cv2(msg, "bgr8")
            fps = 1/(time.time() - self.lastImageT)
            cv2.putText(cvImg,"Image Framerate:" + str(round(fps,2)) ,(40,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            qtImg = convertCVtoQT(self.window, cvImg)
            self.label.setPixmap(qtImg)
            self.lastImageT = time.time()
        except CvBridgeError:
            print("ERROR")


def convertCVtoQT(window, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(int(window.display_width/2), int(window.display_height/2), Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

def fwdFunct():
    wheelControl.commandVel(DEFAULT_SPEED,0)
    rospy.sleep(LINEAR_WAIT)
    wheelControl.commandVel(0,0)
        
def bwdFunct():
    wheelControl.commandVel(-DEFAULT_SPEED,0)
    rospy.sleep(LINEAR_WAIT)
    wheelControl.commandVel(0,0)
    
def leftFunct():
    wheelControl.commandVel(0, -DEFAULT_ANGULAR_SPEED)
    rospy.sleep(RECT_ANGLE_TURN_WAIT)
    wheelControl.commandVel(0,0)

def rightFunct():
    wheelControl.commandVel(0, DEFAULT_ANGULAR_SPEED)
    rospy.sleep(RECT_ANGLE_TURN_WAIT)
    wheelControl.commandVel(0,0)
 
def stopFunct():
    wheelControl.commandVel(0,0)

def ccFunct(butt):
    if not butt.isChecked():
        wheelControl.linearDrive(0)
    else:
        wheelControl.linearDrive(DEFAULT_SPEED)

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

        self.ccCheckBox = QCheckBox("Cruise Control")
        self.ccCheckBox.setChecked(False)
        self.ccCheckBox.stateChanged.connect(lambda:ccFunct(self.ccCheckBox))
        self.layout.addWidget(self.ccCheckBox, 1, 3)

    def treatValue(self):
        if self.ccCheckBox.isChecked():
            wheelControl.commandVel(DEFAULT_SPEED, math.radians(self.directionalSlider.value()))
        else:
            wheelControl.commandVel(0, math.radians(self.directionalSlider.value()))

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

wheelControl = control.CarControl()

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