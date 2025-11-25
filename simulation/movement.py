import pybullet as p
import pybullet_data
import os
import time
import math

# joint mappings (unchanged)
first_joint_leg1    =  9
first_joint_leg2    = 26
first_joint_leg3    = 43

second_joint_leg1   = 13
second_joint_leg2   = 30
second_joint_leg3   = 47

pusher_joint_leg1   = 19
flwr1_pusher_leg1   = 21
flwr2_pusher_leg1   = 22


# Leg map for pusher servos
leg_map = {
    1: (pusher_joint_leg1, flwr1_pusher_leg1, flwr2_pusher_leg1)
}

# Kinematic constants (unchanged)
a = 0.0215918  # link a (m)
b = 0.045699   # link b (m)

def compute_followers(alpha):
    x = (a * math.sin(alpha)) / b
    if x > 1.0:
        x = 1.0
    elif x < -1.0:
        x = -1.0

    phi = math.asin(x)
    return (-(alpha + phi), -phi)

def init_skootr(robotId):
    init_servo_angles = {
        first_joint_leg1:  0.718,
        first_joint_leg2:  0.718,
        first_joint_leg3:  0.718,
        second_joint_leg1: 1.718,
        second_joint_leg2: 1.718,
        second_joint_leg3: 1.718,
    }
    init_pusher_angles = {
        pusher_joint_leg1: 0,
    }

    # Drive all “first” and “second” servos to home
    for joint_idx, target in init_servo_angles.items():
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target,
            force=1e6
        )

    # Drive each leg’s pusher + its two followers to home
    for leg, (pj, f1, f2) in leg_map.items():
        alpha = init_pusher_angles[pj]
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=pj,
            controlMode=p.POSITION_CONTROL,
            targetPosition=alpha,
            force=1e6
        )
        phi1, phi2 = compute_followers(alpha)
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=f1,
            controlMode=p.POSITION_CONTROL,
            targetPosition=phi1,
            force=1e6
        )
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=f2,
            controlMode=p.POSITION_CONTROL,
            targetPosition=phi2,
            force=1e6
        )

def move_servo(robotId, target_joint, target_angle):
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=target_joint,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_angle,
        force=1000
        )
    
def move_pusher(robotId, target_angle):
    leg_index = 1

    pj, f1, f2 = leg_map[leg_index]
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=pj,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_angle,
        force=1000
    )
    phi1, phi2 = compute_followers(target_angle)
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=f1,
        controlMode=p.POSITION_CONTROL,
        targetPosition=phi1,
        force=1000
    )
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=f2,
        controlMode=p.POSITION_CONTROL,
        targetPosition=phi2,
        force=1000
    )

def skooting(robotId):

    # Helper to step the simulation for N frames
    def step_n(n):
        for _ in range(n):
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    # Infinite loop: cycle through Flex → Lift → Extend → Lower for leg 1
    while True:

        move_pusher(robotId, 1.0)
        step_n(120)

        move_servo(robotId, second_joint_leg1, 2.318)
        step_n(120)

        move_pusher(robotId, 0)
        step_n(120)

        move_servo(robotId, second_joint_leg1, 1.7)
        step_n(120)

        move_servo(robotId, first_joint_leg1, 0.5)
        step_n(120)

        step_n(120)

def skating(robotId):
    True

def shuffling(robotId):
    True


def main():
    # — Setup PyBullet —
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

    # Load ground plane & Skootr URDF
    p.loadURDF("plane.urdf", globalScaling=10)
    script_dir = os.path.dirname(__file__)
    urdf_path = os.path.join(script_dir, "skootr_move.urdf")
    robotId = p.loadURDF(urdf_path, basePosition=[0, 0, 0.1], useFixedBase=False)

    # ------------------------------- Debug Links -------------------------------
    # num_joints = p.getNumJoints(robotId)
    # print("Base link (index = -1):", p.getBodyInfo(robotId)[0].decode("utf-8"))
    # for i in range(num_joints):
    #     info = p.getJointInfo(robotId, i)
    #     # info[0] = jointIndex, info[12] = linkName (as bytes)
    #     joint_index = info[0]
    #     link_name   = info[12].decode("utf-8")
    #     print(f"Joint {joint_index} → Link \"{link_name}\"")
    # ------------------------------- Debug Links -------------------------------

    # ------------------------------- Debug Joints -------------------------------
    # num_joints = p.getNumJoints(robotId)
    # print(f"Total joints = {num_joints}\n")
    # for i in range(num_joints):
    #     info = p.getJointInfo(robotId, i)
    #     joint_index = info[0]                                # int
    #     joint_name  = info[1].decode("utf-8")                 # bytes → str
    #     joint_type  = info[2]                                # int code


    #     print(f"Joint {joint_index:2d} : {joint_name:<25s}  type = {joint_type}")
    # ------------------------------- Debug Joints -------------------------------


    # Init skootr
    init_skootr(robotId)
    for _ in range(50):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    # Simulation for Gait 1, skooting
    # skooting(robotId)

    # # Simulation for Gait 2, skating
    # skating(robotId)

    # # Simulation for Gait 3, shuffling
    # shuffling(robotId)

    while True:
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

if __name__ == "__main__":
    main()
