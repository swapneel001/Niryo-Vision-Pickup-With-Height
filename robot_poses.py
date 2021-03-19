#Defining co-ordinates of poses for workspace 
pose1 = PoseObject(
    x=0.669, y=0.166, z=0.002,
    roll=-2.602, pitch=0.96, yaw=0.500,
)
pose2 = PoseObject(
    x=0.679, y=-0.028, z=0.002,
    roll=3.134, pitch=1.115, yaw=-0.071,
)
pose3 = PoseObject(
    x=0.187, y=-0.028, z=0.002,
    roll=2.833, pitch=1.471, yaw=2.269,
)
pose4 = PoseObject(
    x=0.191, y=0.166, z=0.002,
    roll=-0.215, pitch=1.078, yaw=2.269,
)

drop_pose = PoseObject(  # position for the robot to place object
    x=0.038, y=-0.107, z=0.136,
    roll=-0.084, pitch=1.428, yaw=-1.194,
)
observation_pose = PoseObject(  # position for the robot to watch the workspace
    x=0.111, y=0.007, z=0.225,
    roll=0.255, pitch=0.992, yaw=0.070,
)
