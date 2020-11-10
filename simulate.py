from nextstate import getnextState
from feedForward import feedForwardControl
from math import pi
from trajgen import generateTrajectory
from youBot import ControlVector, endEffectorinSpace, flatTrajtoTransform, youBotState
import numpy as np
import matplotlib.pyplot as plt
import modern_robotics as mr

initialState = youBotState()
initialState.jointState = np.array([0.,0,0.3,-1.6,0])
currentState = initialState
controls = ControlVector()
Kp =  0* np.eye(6)
Ki = 0* np.eye(6)

initCube = [1,0,0]
finalCube = [0,-1,-pi/2]
stateMatrix = []
Error = []

######### Generate Trajectory ########
Trajectory = generateTrajectory(initialState,initCube,finalCube)
np.savetxt('traj.csv',Trajectory,fmt="%f",delimiter=",")
# trajectory defined

### Simulating the stuff ####

for i in range( len(Trajectory) -1 ):

    update = np.concatenate((currentState.chasisState,currentState.jointState,
                currentState.wheelState,currentState.gripperState),axis=None)
    stateMatrix.append(update)

    X = endEffectorinSpace(currentState)
    Xd,gripper = flatTrajtoTransform(Trajectory[i])
    XdNext,_gripper = flatTrajtoTransform(Trajectory[i+1])

    currentState.gripperState = np.array(gripper)
    Xrr = feedForwardControl(X,Xd,XdNext,currentState,controls,Kp,Ki)
    Error.append(Xrr)
    getnextState(currentState,controls)
Error = np.array(Error)
stateMatrix = np.array(stateMatrix)
np.savetxt('stateMat.csv',stateMatrix,fmt='%f',delimiter=',')

### plot the results
iterations = range(len(Trajectory)-1)

plt.plot(iterations,Error[:,0])
plt.plot(iterations,Error[:,1])
plt.plot(iterations,Error[:,2])    
plt.plot(iterations,Error[:,3])
plt.plot(iterations,Error[:,4])
plt.plot(iterations,Error[:,5])
plt.show()
plt.close('all')


