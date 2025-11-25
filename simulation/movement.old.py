import pybullet as p
import pybullet_data
import os
import time
import math

# joint mappings
first_joint_leg1    =  9
first_joint_leg2    = 26
first_joint_leg3    = 43

second_joint_leg1   = 13
second_joint_leg2   = 30
second_joint_leg3   = 47

pusher_joint_leg1   = 19
flwr1_pusher_leg1   = 21
flwr2_pusher_leg1   = 22

pusher_joint_leg2   = 36
flwr1_pusher_leg2   = 38
flwr2_pusher_leg2   = 39

pusher_joint_leg3   = 53
flwr1_pusher_leg3     = 55
flwr2_pusher_leg3     = 56

# Kinematic constants
a = 0.0215918  # link a (m)
b = 0.045699   # link b (m)

def compute_followers(alpha):
    # guard domain for asin
    x = a * math.sin(alpha) / b
    if abs(x) > 1.0:
        phi = 0.0
    else:
        phi = math.asin(x)
    return -(alpha + phi), -phi

def init_skootr(robotId):
    # Initialize
    init_servo_angles = {
        first_joint_leg1: 0.718,
        first_joint_leg2: 0.718,
        first_joint_leg3: 0.718,
        second_joint_leg1: 1.718,
        second_joint_leg2: 1.718,
        second_joint_leg3: 1.718,
    }
    init_pusher_angles = {
        pusher_joint_leg1: 1.5,
        pusher_joint_leg2: 1.5,
        pusher_joint_leg3: 1.5
    }

    # Drive servos to init positions
    for joint_idx, target in init_servo_angles.items():
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target,
            force=1000
        )

    # Drive pusher servos and followers to init positions
    leg_map = {
        1: (pusher_joint_leg1, flwr1_pusher_leg1, flwr2_pusher_leg1),
        2: (pusher_joint_leg2, flwr1_pusher_leg2, flwr2_pusher_leg2),
        3: (pusher_joint_leg3, flwr1_pusher_leg3, flwr2_pusher_leg3),
    }

    for leg, (pj, f1, f2) in leg_map.items():
        alpha = init_pusher_angles[pj]
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=pj,
            controlMode=p.POSITION_CONTROL,
            targetPosition=alpha,
            force=1000
        )
        # compute & drive followers
        phi1, phi2 = compute_followers(alpha)
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
    

def main():
    # Setup
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.resetDebugVisualizerCamera(1.0, 50, -30, [0, 0, 0])

    # Load plane & robot
    p.loadURDF("plane.urdf", globalScaling=10)
    script_dir = os.path.dirname(__file__)
    urdf_path = os.path.join(script_dir, "skootr.urdf")
    robotId = p.loadURDF(urdf_path, basePosition=[0, 0, 0.1], useFixedBase=True)
    
    # Run
    while True:
        init_skootr(robotId)


if __name__ == "__main__":
    main()
