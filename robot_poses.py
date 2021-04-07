from niryo_one_tcp_client import *
from niryo_one_camera import *

#Defining co-ordinates of poses for workspace 
pose1 = PoseObject(
    x=0.718, y=0.194, z=0.00,
    roll=3.094, pitch=1.519, yaw=3.06,
)
pose2 = PoseObject(
    x=0.718, y=-0.006, z=0.00,
    roll=3.094, pitch=1.519, yaw=3.06,
)
pose3 = PoseObject(
    x=0.177, y=-0.006, z=0.00,
    roll=3.094, pitch=1.519, yaw=3.06,
)
pose4 = PoseObject(
    x=0.177, y=0.194, z=0.00,
    roll=3.094, pitch=1.519, yaw=3.06,
)
drop_pose1 = PoseObject( # position for the robot to place object in dustbin 1
    x = -0.019, y = -0.347, z = 0.063,
    roll = -1.671, pitch = 1.525, yaw = 3.065
)
drop_pose2 = PoseObject(  # position for the robot to place object in dustbin 2
    x = -0.244, y = -0.277, z = 0.087,
    roll = 0.087, pitch = 1.142, yaw = -2.255
)
observation_pose = PoseObject(  # position for the robot to watch the workspace
    x=0.067, y=0.004, z=0.197,
    roll=0.238, pitch=1.034, yaw=0.067,
)