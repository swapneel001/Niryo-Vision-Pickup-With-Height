from niryo_one_tcp_client import *
from niryo_one_camera import *

#Defining co-ordinates of poses for workspace 
pose1 = PoseObject(
    x=0.725, y=0.181, z=-0.004,
    roll=3.12, pitch=1.444, yaw=3.032,
)
pose2 = PoseObject(
    x=0.725, y=-0.019, z=-0.004,
    roll=3.12, pitch=1.444, yaw=3.032,
)
pose3 = PoseObject(
    x=0.184, y=-0.019, z=-0.004,
    roll=3.12, pitch=1.444, yaw=3.032,
)
pose4 = PoseObject(
    x=0.184, y=0.181, z=-0.004,
    roll=3.12, pitch=1.444, yaw=3.032,
)

drop_pose = PoseObject(  # position for the robot to place object
    x=0.031, y=-0.245, z=0.168,
    roll=0, pitch=1.57, yaw=0,
)
observation_pose = PoseObject(  # position for the robot to watch the workspace
    x=0.067, y=0.004, z=0.197,
    roll=0.238, pitch=1.034, yaw=0.067,
)