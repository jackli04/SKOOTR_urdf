import pybullet as p
import pybullet_data
import os, time



# GUI
physicsClient = p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
p.resetSimulation()
p.setGravity(0, 0, -9.81)

p.resetDebugVisualizerCamera(
    cameraDistance=1.0, 
    cameraYaw=  50.0, 
    cameraPitch=-30.0,
    cameraTargetPosition=[0,0,0]
)

# Load a ground plane
planeId = p.loadURDF("plane.urdf", globalScaling=10)

# URDF model
script_dir = os.path.dirname(os.path.realpath(__file__))
urdf_path = os.path.join(script_dir, "skootr_simplified.urdf")
robotId = p.loadURDF(urdf_path,
                     basePosition=[0, 0, .1],
                     useFixedBase=True)

# Inspect joints
num_joints = p.getNumJoints(robotId)
print(f"Number of joints: {num_joints}")
for ji in range(num_joints):
    info = p.getJointInfo(robotId, ji)
    print(f"Joint {ji}: name={info[1].decode()} type={info[2]}")

sliders = []
for jid in range(p.getNumJoints(robotId)):
    info = p.getJointInfo(robotId, jid)
    jtype = info[2]
    name  = info[1].decode()
    if jtype == 0:
        slider_id = p.addUserDebugParameter(name, -3.14, 3.14, 0.0)
        sliders.append((jid, slider_id))
    if jtype == 1:
        slider_id = p.addUserDebugParameter(name, -0, 0.0215, 0,0)
        sliders.append((jid, slider_id))

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
    p.stepSimulation()
    time.sleep(1.0/240.0)
