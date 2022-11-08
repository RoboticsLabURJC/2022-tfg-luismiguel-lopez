from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QPushButton, QGridLayout, QSlider
from PyQt5.QtGui import QPixmap
from PyQt5 import QtGui
import numpy as np

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
import cv2

import robotcontrol

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
        fwdButton = QPushButton("Forward")
        #openButton.clicked.connect(openFunct)
        self.layout.addWidget(fwdButton, 1, 0)

        bwdButton = QPushButton("Backwards")
        #closeButton.clicked.connect(closeFunct)
        self.layout.addWidget(bwdButton, 1, 2)

        stopButton = QPushButton("Stop") 
        #stopButton.clicked.connect(stopFunct)
        self.layout.addWidget(stopButton, 1, 1)

        leftButton = QPushButton("Left") 
        #csvButton.clicked.connect(storeCsvFunct)
        self.layout.addWidget(leftButton, 0, 1)

        rightButton = QPushButton("Right") 
        #csvButton.clicked.connect(storeCsvFunct)
        self.layout.addWidget(rightButton, 2, 1)

    def addImages(self):
        self.window.image = QLabel("Camera View")
        self.layout.addWidget(self.window.image, 3, 0)

    def executeInterface(self):
        self.window.setLayout(self.layout)
        self.window.show()
        self.app.exec_()


rospy.init_node('interface_node')

wheelControl = robotcontrol.carControl()

interface = InterfaceWindow()
interface.addButtons()
interface.addImages()

image = Image(interface.window)

def main():

    interface.executeInterface()

if __name__ == '__main__':
    main()