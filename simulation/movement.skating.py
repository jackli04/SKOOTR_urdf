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
        upper_joint_leg1:  0.718,
        upper_joint_leg2:  0.718,
        upper_joint_leg3:  0.718,
        lower_joint_leg1: 1.818,
        lower_joint_leg2: 1.818,
        lower_joint_leg3: 1.818,
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
            force=50
        )

    # Drive pusher to home
    for joint_idx, target in init_pusher_angles.items():
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target,
            force=50
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
    urdf_path = os.path.join(script_dir, "skootr_simplified.urdf")
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

    # Simulate skating

    # Lift pusher for skating
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=pusher_leg1,
        controlMode=p.POSITION_CONTROL,
        targetPosition=0.0215,
        force=50
    )

    # Skating loop: extend then retract leg1
    extend_target = {upper_joint_leg1: 1.1, lower_joint_leg1: 1.0}
    retract_target = {upper_joint_leg1: 0.618, lower_joint_leg1: 2.118}

    while True:
        # extend phase
        for joint, tgt in extend_target.items():
            p.setJointMotorControl2(
                bodyIndex=robotId,
                jointIndex=joint,
                controlMode=p.POSITION_CONTROL,
                targetPosition=tgt,
                force=50
            )
        for _ in range(80):
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=upper_joint_leg1,
            controlMode=p.POSITION_CONTROL,
            targetPosition=0.8,
            force=50
        )
        for _ in range(10):
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
        # retract phase
        for joint, tgt in retract_target.items():
            p.setJointMotorControl2(
                bodyIndex=robotId,
                jointIndex=joint,
                controlMode=p.POSITION_CONTROL,
                targetPosition=tgt,
                force=50
            )
        for _ in range(80):
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

if __name__ == "__main__":
    main()
