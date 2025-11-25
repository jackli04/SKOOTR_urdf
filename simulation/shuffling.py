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

pusher_joint_leg2   = 36
flwr1_pusher_leg2   = 38
flwr2_pusher_leg2   = 39

pusher_joint_leg3   = 53
flwr1_pusher_leg3   = 55
flwr2_pusher_leg3   = 56

# Leg map for pusher servos
leg_map = {
    1: (pusher_joint_leg1, flwr1_pusher_leg1, flwr2_pusher_leg1),
    2: (pusher_joint_leg2, flwr1_pusher_leg2, flwr2_pusher_leg2),
    3: (pusher_joint_leg3, flwr1_pusher_leg3, flwr2_pusher_leg3),
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
        pusher_joint_leg1: 0.0,
        pusher_joint_leg2: 0.0,
        pusher_joint_leg3: 0.0,
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

def move_pusher(robotId, target_leg, target_angle):
    # Resolve which pusher joint index and follower indices belong to this leg
    if target_leg == pusher_joint_leg1:
        leg_index = 1
    elif target_leg == pusher_joint_leg2:
        leg_index = 2
    else:
        leg_index = 3

    pj, f1, f2 = leg_map[leg_index]
    # Drive the “pusher” joint
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=pj,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_angle,
        force=1000
    )
    # Compute the two follower angles from α 
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
    urdf_path = os.path.join(script_dir, "skootr.urdf")
    robotId = p.loadURDF(urdf_path, basePosition=[0, 0, 0.1], useFixedBase=False)

    init_skootr(robotId)

    to_lock = [
        36,  # pusher_joint_leg2
        38,  # flwr1_pusher_leg2
        39,  # flwr2_pusher_leg2
        53,  # pusher_joint_leg3
        55,  # flwr1_pusher_leg3
        56   # flwr2_pusher_leg3
    ]
    for j in to_lock:
        info = p.getJointInfo(robotId, j)
        parent_link = info[16]      # parent link index
        parent_pos  = info[14]      # joint origin position in parent’s frame
        parent_orn  = info[15]      # joint origin orientation in parent’s frame (quaternion)
        p.createConstraint(
            parentBodyUniqueId      = robotId,
            parentLinkIndex         = parent_link,
            childBodyUniqueId       = robotId,
            childLinkIndex          = j,
            jointType               = p.JOINT_FIXED,
            jointAxis               = [0, 0, 0],
            parentFramePosition     = parent_pos,
            childFramePosition      = [0, 0, 0],
            parentFrameOrientation  = parent_orn,
            childFrameOrientation   = [0, 0, 0, 1]
        )

    # Let everything settle for a moment
    for _ in range(50):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    # — Shuffling parameters —
    α_ext   =  0.50    # “extend” angle (radians)
    α_flex  = -0.50    # “flex”    angle (radians)
    α_neu   =  0.00    # neutral (leg down on ground)

    phase_duration = 0.25     # each sub‐phase lasts 0.25 s
    phase_steps    = int(phase_duration * 240)  # 240 Hz simulation

    # A helper to step the sim for N frames
    def step_n(n):
        for _ in range(n):
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    # Main loop: cycle through leg 1 → leg 2 → leg 3
    leg_sequence = [1, 2, 3]
    while True:
        for leg_idx in leg_sequence:
            # 1) Extend
            move_pusher(robotId, 
                        leg_map[leg_idx][0],  # pusher_joint index for this leg
                        α_ext)
            step_n(phase_steps)

            # 2) Lower (back to neutral, i.e. place foot back on the ball)
            move_pusher(robotId, 
                        leg_map[leg_idx][0],
                        α_neu)
            step_n(phase_steps)

            # 3) Flex (bring the leg “rearward” or off‐ground)
            move_pusher(robotId, 
                        leg_map[leg_idx][0],
                        α_flex)
            step_n(phase_steps)

            # 4) Lift (back to neutral so foot touches again)
            move_pusher(robotId, 
                        leg_map[leg_idx][0],
                        α_neu)
            step_n(phase_steps)

        # After finishing legs 1→2→3 you can loop again to keep shuffling forever.

if __name__ == "__main__":
    main()
