import pybullet as p
import pybullet_data
import os, time, math

servo_to_horn_leg1 = 9
servo_to_horn_leg2 = 26
servo_to_horn_leg3 = 43

servo_joint_index_leg1 = 19
follower1_index_leg1 = 21
follower2_index_leg1 = 22

servo_joint_index_leg2 = 36
follower1_index_leg2 = 38
follower2_index_leg2 = 39

servo_joint_index_leg3 = 53
follower1_index_leg3 = 55
follower2_index_leg3 = 56


# Connect to GUI
physicsClient = p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
p.resetSimulation()
p.setGravity(0, 0, -9.81)

p.resetDebugVisualizerCamera(
    cameraDistance=1.0, 
    cameraYaw=50.0, 
    cameraPitch=-30.0,
    cameraTargetPosition=[0, 0, 0]
)

# Load ground plane
planeId = p.loadURDF("plane.urdf", globalScaling=10)

# Load URDF robot
script_dir = os.path.dirname(os.path.realpath(__file__))
urdf_path = os.path.join(script_dir, "skootr.urdf")
robotId = p.loadURDF(urdf_path, basePosition=[0, 0, 0.1], useFixedBase=True)

# Inspect joints
num_joints = p.getNumJoints(robotId)
print(f"Number of joints: {num_joints}")
for ji in range(num_joints):
    info = p.getJointInfo(robotId, ji)
    print(f"Joint {ji}: name={info[1].decode()} type={info[2]}")

# Add user sliders for revolute joints
sliders = []
skip_joints = [follower1_index_leg1, follower2_index_leg1, follower1_index_leg2, follower2_index_leg2, follower1_index_leg3, follower2_index_leg3]
for jid in range(p.getNumJoints(robotId)):
    info = p.getJointInfo(robotId, jid)
    jtype = info[2]
    name = info[1].decode()
    if jtype == p.JOINT_REVOLUTE and jid not in skip_joints:
        if jid in [servo_joint_index_leg1, servo_joint_index_leg2, servo_joint_index_leg3]:
            slider_id = p.addUserDebugParameter(name, 0, 1.5, 0.0)
        elif jid in [servo_to_horn_leg1, servo_to_horn_leg2, servo_to_horn_leg3]:
            slider_id = p.addUserDebugParameter(name, 0, 1.187, 0.0)
        else:
            slider_id = p.addUserDebugParameter(name, -2.356, 2.356, 0.0)

        sliders.append((jid, slider_id))

# Constants for calculations
a = 0.0215918  # in meters
b = 0.045699   # in meters

# Joint indices




# Main simulation loop
while True:
    for jid, slider_id in sliders:
        angle = p.readUserDebugParameter(slider_id)
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=jid,
            controlMode=p.POSITION_CONTROL,
            targetPosition=angle,
            force=200
        )

    # pusher for first leg
    servo_angle_leg1 = p.readUserDebugParameter([s[1] for s in sliders if s[0] == servo_joint_index_leg1][0])
    # Control servo joint
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=servo_joint_index_leg1,
        controlMode=p.POSITION_CONTROL,
        targetPosition=servo_angle_leg1,
        force=200
    )
    try:
        sin_alpha = math.sin(servo_angle_leg1)
        if sin_alpha == 0:
            follower_degree_leg1 = 0
        else:
            follower_degree_leg1 = math.asin(a * sin_alpha / b)
    except (ValueError, ZeroDivisionError):
        follower_degree = 0  # Handle edge cases
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=follower1_index_leg1,
        controlMode=p.POSITION_CONTROL,
        targetPosition=-(servo_angle_leg1+follower_degree_leg1),
        force=200
    )
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=follower2_index_leg1,
        controlMode=p.POSITION_CONTROL,
        targetPosition=-follower_degree_leg1,
        force=200
    )


    # pusher for second leg
    servo_angle_leg2 = p.readUserDebugParameter([s[1] for s in sliders if s[0] == servo_joint_index_leg2][0])
    # Control servo joint
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=servo_joint_index_leg2,
        controlMode=p.POSITION_CONTROL,
        targetPosition=servo_angle_leg2,
        force=200
    )
    try:
        sin_alpha = math.sin(servo_angle_leg2)
        if sin_alpha == 0:
            follower_degree_leg2 = 0
        else:
            follower_degree_leg2 = math.asin(a * sin_alpha / b)
    except (ValueError, ZeroDivisionError):
        follower_degree = 0  # Handle edge cases
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=follower1_index_leg2,
        controlMode=p.POSITION_CONTROL,
        targetPosition=-(servo_angle_leg2+follower_degree_leg2),
        force=200
    )
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=follower2_index_leg2,
        controlMode=p.POSITION_CONTROL,
        targetPosition=-follower_degree_leg2,
        force=200
    )


    # pusher for second leg
    servo_angle_leg3 = p.readUserDebugParameter([s[1] for s in sliders if s[0] == servo_joint_index_leg3][0])
    # Control servo joint
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=servo_joint_index_leg3,
        controlMode=p.POSITION_CONTROL,
        targetPosition=servo_angle_leg3,
        force=200
    )
    try:
        sin_alpha = math.sin(servo_angle_leg3)
        if sin_alpha == 0:
            follower_degree_leg3 = 0
        else:
            follower_degree_leg3 = math.asin(a * sin_alpha / b)
    except (ValueError, ZeroDivisionError):
        follower_degree = 0  # Handle edge cases
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=follower1_index_leg3,
        controlMode=p.POSITION_CONTROL,
        targetPosition=-(servo_angle_leg3+follower_degree_leg3),
        force=200
    )
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=follower2_index_leg3,
        controlMode=p.POSITION_CONTROL,
        targetPosition=-follower_degree_leg3,
        force=200
    )


    # print("GUI angle: ", servo_angle, ", follower degree: ", follower_degree, '\n')

    p.stepSimulation()
    time.sleep(1.0/480.0)
