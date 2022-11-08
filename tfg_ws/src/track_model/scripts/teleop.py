from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QPushButton, QVBoxLayout, QGridLayout, QSlider
from PyQt5.QtGui import QPixmap, QImage, QPainter
from PyQt5 import QtGui
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread, QPointF, QRectF, QLineF
import numpy as np

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import csv

from enum import Enum

bridge = CvBridge()

class Image:
    def __init__(self, interface):
        self.topic = "/camera/zed/rgb/image_rect_color"
        self.sub = rospy.Subscriber(self.topic, Image, self.cb)
        self.window = interface
        self.label = interface.chassisImgLabel

    def cb(self, msg):
        try:
        # Convert your ROS Image message to OpenCV2
            cvImg = bridge.imgmsg_to_cv2(msg, "bgr8")
            qtImg = convertCVtoQT(self.window, cvImg)
            self.label.setPixmap(qtImg)
        except CvBridgeError:
            print("ERROR")

class WheelController:
    def __init__(self):
        self.wheel1Pub = rospy.Publisher("/robot/left_rear_controller/command", Float64, queue_size=10)
        self.wheel2Pub = rospy.Publisher("/robot/left_front_controller/command", Float64, queue_size=10)
        self.leftSteeringController = rospy.Publisher("/robot/right_rear_controller/command", Float64, queue_size=10)
        self.leftSteeringController = rospy.Publisher("/robot/right_rear_controller/command", Float64, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
        self.commandedSpeed = 0

    def publish_all_wheels(self, value):
        msg = Float64()
        msg.data = value
        self.wheel1Pub.publish(msg)
        self.wheel2Pub.publish(msg)
        self.wheel3Pub.publish(msg)
        self.wheel4Pub.publish(msg)
        self.commandedSpeed = value

    def stop(self):
        msg = Float64()
        msg.data = 0.0
        self.wheel1Pub.publish(msg)
        self.wheel2Pub.publish(msg)
        self.wheel3Pub.publish(msg)
        self.wheel4Pub.publish(msg)
        self.commandedSpeed = 0

    def getActualSpeed(self):
        return self.commandedSpeed

    def sleep(self):
        self.rate.sleep()

def moveFunct(value):
    wheelControl.publish_all_wheels(value)
    wheelControl.sleep()

def stopFunct():
    wheelControl.stop()
    wheelControl.sleep()

def convertCVtoQT(window, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(int(window.display_width/4), int(window.display_height/4), Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

class InterfaceWindow():
    def __init__(self):
        self.app = QApplication([])
        self.window = QWidget()
        self.layout = QGridLayout()
        self.window.setWindowTitle("Controller Interface")
        self.window.display_width = 640
        self.window.display_height = 480

    def addButtons(self):
        openButton = QPushButton("Open Gripper")
        openButton.clicked.connect(openFunct)
        self.layout.addWidget(openButton, 0, 4)

        closeButton = QPushButton("Close Gripper")
        closeButton.clicked.connect(closeFunct)
        self.layout.addWidget(closeButton, 1, 4)

        stopButton = QPushButton("STOP CAR") 
        stopButton.clicked.connect(stopFunct)
        self.layout.addWidget(stopButton, 0, 2)

        csvButton = QPushButton("Save CSV") 
        csvButton.clicked.connect(storeCsvFunct)
        self.layout.addWidget(csvButton, 3, 0)

        armControlJoystick = Joystick()

        self.layout.addWidget(armControlJoystick, 1, 2)

    def treatValue(self):
        moveFunct(float(self.speedSlider.value()))

    def addSliders(self):
        self.speedSlider = QSlider(Qt.Vertical)
        self.speedSlider.setMinimum(-5)
        self.speedSlider.setMaximum(5)
        self.speedSlider.setValue(0)
        self.speedSlider.setTickPosition(QSlider.TicksBelow)
        self.speedSlider.setTickInterval(1)
        self.layout.addWidget(self.speedSlider, 0, 1)
        self.speedSlider.valueChanged.connect(self.treatValue)

    def addImages(self):

        self.window.gripperImgLabel = QLabel("Gripper Image")
        self.window.chassisImgLabel = QLabel("Chassis Image")
        self.window.filteredImgLabel = QLabel("Green Filter Image")
        self.window.isDetected = QLabel("Green Cube Detected: ")

        self.layout.addWidget(self.window.gripperImgLabel, 0, 0)
        self.layout.addWidget(self.window.chassisImgLabel, 1, 0)
        self.layout.addWidget(self.window.filteredImgLabel, 2, 0)
        self.layout.addWidget(self.window.isDetected, 2, 2)

    def executeInterface(self):
        self.window.setLayout(self.layout)
        self.window.show()
        self.app.exec_()


rospy.init_node('interface_node')

wheelControl = WheelController()

interface = InterfaceWindow()
interface.addButtons()
interface.addImages()

chassisImg = ChassisCameraImage(interface.window)

interface.addSliders()


def main():

    interface.executeInterface()

if __name__ == '__main__':
    main()