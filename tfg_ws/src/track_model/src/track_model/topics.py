#!/user/bin/env python3
CAMERA_TOPIC = "/camera/image"

# case direct communication with gazebo is used
RIGHT_REAR_TOPIC = "/racecar/right_rear_wheel_velocity_controller/command"
LEFT_REAR_TOPIC = "/racecar/left_rear_wheel_velocity_controller/command"
RIGHT_FRONT_TOPIC = "/racecar/right_front_wheel_velocity_controller/command"
LEFT_FRONT_TOPIC = "/racecar/left_front_wheel_velocity_controller/command"

LEFT_DIRECTION_TOPIC = "/racecar/left_steering_hinge_position_controller/command"
RIGHT_DIRECTION_TOPIC = "/racecar/right_steering_hinge_position_controller/command"

# case wrapper is used

COMMANDED_DATA_TOPIC = "/cmd_vel"