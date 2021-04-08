import pybullet as p
import numpy as np
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
basePos = [0,0,0]

# import os
# os.system("C:/Users/nwinn/Desktop/Spring2021/KDC/kuka_experimental-indigo-devel")

# robot = p.loadURDF("kuka_experimental-indigo-devel/kuka_kr210_support/urdf/kr210l150.urdf", startPos, startOrientation, useFixedBase=1)

quadrotor = p.loadURDF("quadrotor_small/quadrotor.urdf")

# p.setJointMotorControlArray(
#     robot, range(6), p.POSITION_CONTROL,
#     targetPositions=[0.1] * 6)
# print(p.getNumJoints(quadrotor))

curPos = p.getBasePositionAndOrientation(quadrotor)

# boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
p.applyExternalForce(quadrotor, -1, [0,0,100], [0,0,0],p.LINK_FRAME)
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
    curPos = p.getBasePositionAndOrientation(quadrotor)
    print(curPos[0][2])
    if (curPos[0][2]) < 1:
        p.applyExternalForce(quadrotor, -1, [0,0,5], [0,0,0],p.LINK_FRAME)
    else:
        p.applyExternalForce(quadrotor, -1, [0,0,4], [0,0,0], p.LINK_FRAME)
    # p.setJointMotorControlArray(
    #     robot, range(6), p.POSITION_CONTROL,
    #     targetPositions=[0.1*i/100, 0, 0, np.sin(i/100), np.sin(i/100), 0])
cubePos, cubeOrn = p.getBasePositionAndOrientation(quadrotor)
print(cubePos,cubeOrn)
p.disconnect()
