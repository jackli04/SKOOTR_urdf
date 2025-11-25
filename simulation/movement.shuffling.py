import pybullet as p
import pybullet_data
import os, time


# joint mappings (unchanged)
upper_joint_leg1    =  9
upper_joint_leg2    = 23
upper_joint_leg3    = 37

lower_joint_leg1    = 13
lower_joint_leg2    = 27
lower_joint_leg3    = 41

pusher_leg1         = 19
pusher_leg2         = 33
pusher_leg3         = 47

# Links
wheels_leg1         = 20
wheels_leg2         = 34
wheels_leg3         = 48

def init_skootr(robotId):
    init_servo_angles = {
        upper_joint_leg1:  0.918,
        upper_joint_leg2:  0.918,
        upper_joint_leg3:  0.918,
        lower_joint_leg1: 1.718,
        lower_joint_leg2: 1.718,
        lower_joint_leg3: 1.718,
    }
    init_pusher_angles = {
        pusher_leg1: 0,
        pusher_leg2: 0,
        pusher_leg3: 0
    }

    # Drive all upper and lower servos to home
    for joint_idx, target in init_servo_angles.items():
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target,
            force=5000
        )

    # Drive pusher to home
    for joint_idx, target in init_pusher_angles.items():
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target,
            force=5000
        )


def main():
    # Setup PyBullet
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
    planeId = p.loadURDF("plane.urdf", globalScaling=10)
    script_dir = os.path.dirname(__file__)
    urdf_path = os.path.join(script_dir, "skootr_heavy_base.urdf")
    robotId = p.loadURDF(urdf_path, basePosition=[0, 0, 0.1], useFixedBase=False)

    # p.changeDynamics(
    #     bodyUniqueId=planeId,
    #     linkIndex=-1,
    #     lateralFriction=0.2,
    #     restitution=0.0
    # )   
    p.changeDynamics(
        bodyUniqueId=robotId,
        linkIndex=wheels_leg1,
        lateralFriction=1.0,        # Coulomb μ
        spinningFriction=0.1,       # resists spinning about the wheel axis
        rollingFriction=0.01,       # resists pure rolling slip
        restitution=0.0             # bounciness
    )
    p.changeDynamics(
        bodyUniqueId=robotId,
        linkIndex=wheels_leg2,
        lateralFriction=1.0,        # Coulomb μ
        spinningFriction=0.1,       # resists spinning about the wheel axis
        rollingFriction=0.01,       # resists pure rolling slip
        restitution=0.0             # bounciness
    )
    p.changeDynamics(
        bodyUniqueId=robotId,
        linkIndex=wheels_leg3,
        lateralFriction=1.0,        # Coulomb μ
        spinningFriction=0.1,       # resists spinning about the wheel axis
        rollingFriction=0.01,       # resists pure rolling slip
        restitution=0.0             # bounciness
    )

    num_joints = p.getNumJoints(robotId)
    print("Total joints (links):", num_joints)
    for idx in range(num_joints):
        info = p.getJointInfo(robotId, idx)
        joint_index = info[0]
        joint_name  = info[1].decode('utf-8')
        link_name   = info[12].decode('utf-8')
        print(f"Index {joint_index}: joint='{joint_name}', link='{link_name}'")


    # Init skootr
    init_skootr(robotId)
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

   # Time step for simulation
    dt = 1.0 / 360.0

    # Warm-up simulation
    for _ in range(100):
        p.stepSimulation()
        time.sleep(dt)

    # --- Shuffling phase ---
    while True:
        # Retract pusher
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=pusher_leg1,
            controlMode=p.POSITION_CONTROL,
            targetPosition=0.0215,
            force=5000
        )
        for _ in range(150):
            p.stepSimulation()
            time.sleep(dt)

        # Extend upper leg and lower leg
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=upper_joint_leg1,
            controlMode=p.POSITION_CONTROL,
            targetPosition=1.0,
            force=5000
        )
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=lower_joint_leg1,
            controlMode=p.POSITION_CONTROL,
            targetPosition=1.5,
            force=5000
        )
        for _ in range(200):
            p.stepSimulation()
            time.sleep(dt)

        # Extend pusher back to 0
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=pusher_leg1,
            controlMode=p.POSITION_CONTROL,
            targetPosition=0.0,
            force=5000
        )
        for _ in range(150):
            p.stepSimulation()
            time.sleep(dt)

        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=upper_joint_leg1,
            controlMode=p.POSITION_CONTROL,
            targetPosition=0.918,
            force=5000
        )
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=lower_joint_leg1,
            controlMode=p.POSITION_CONTROL,
            targetPosition=1.718,
            force=5000
        )
        for _ in range(300):
            p.stepSimulation()
            time.sleep(dt)

if __name__ == "__main__":
    main()
