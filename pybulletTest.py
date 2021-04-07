import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
basePos = [0,0,0]

import os
os.system("C:/Users/nwinn/Desktop/Spring2021/KDC/kuka_experimental-indigo-devel")

robot = p.loadURDF("kuka_experimental-indigo-devel/kuka_kr210_support/urdf/kr210l150.urdf", startPos, startOrientation, useFixedBase=1)

# p.setJointMotorControlArray(
#     robot, range(6), p.POSITION_CONTROL,
#     targetPositions=[0.1] * 6)

# boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()
