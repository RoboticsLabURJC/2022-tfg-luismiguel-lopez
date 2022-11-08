import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

class carControl:
    def __init__(self):
        self.rightRearPub = rospy.Publisher("/robot/right_rear_wheel_joint/command", Float64, queue_size=10)
        self.leftRear2Pub = rospy.Publisher("/robot/left_rear_wheel_joint/command", Float64, queue_size=10)
        self.leftSteeringPub = rospy.Publisher("/robot/left_steering_hinge_joint/command", Float64, queue_size=10)
        self.rightSteeringPub = rospy.Publisher("/robot/right_steering_hinge_joint/command", Float64, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

    def drive(self, v, w):
        print("Targeted v " + str(v))
        print("Targeted w " + str(w))
