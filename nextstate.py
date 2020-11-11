from youBot import ControlVector, basePosition, youBotProperties, youBotState
import numpy as np
import modern_robotics as mr
from math import acos, atan2, sin,cos


def getnextState(currentState,controls):
    """
    takes the current state of the robot and calculates the next state.
    based on odomentery for chasis and First order euler intergration for joint of the manipulator

    input:
    current state: instance of the youBot state class
    controlVector : instance of the youBot control vector class

    returns:
    the state of the robot at time t.

    """
    controls.jointSpeeds = youBotProperties.saturate(controls.jointSpeeds,youBotProperties.jointSpeedLimit)
    controls.wheelSpeeds = youBotProperties.saturate(controls.wheelSpeeds,youBotProperties.wheelSpeedLimit)


    currentState.jointState = currentState.jointState + youBotProperties.deltaT * controls.jointSpeeds
    temp = currentState.wheelState + youBotProperties.deltaT * controls.wheelSpeeds

    deltaTheta = temp - currentState.wheelState
    
    currentState.wheelState = temp
    Vb6 = np.dot( youBotProperties.F6 , deltaTheta)

    Tbb_ = mr.MatrixExp6(mr.VecTose3(Vb6))
    Tsbnew = np.dot(basePosition(currentState.chasisState),Tbb_)

    phi,x,y = atan2(Tsbnew[1,0],Tsbnew[0,0]),Tsbnew[0,3],Tsbnew[1,3]
    currentState.chasisState = np.array([phi,x,y])
    # wZ , vX , vY = Vb
    
    
    # if mr.NearZero(wZ):
    #     qb = np.array([0,vX,vY])
    # else:
    #     qb = np.array([wZ, vX * sin(wZ) + (vY/wZ)*( cos(wZ)-1 ) , vY*sin(wZ) + (vX/wZ)*(1-cos(wZ)) ])
    
    # deltaQ = np.dot(np.array([
    #     [1,0,0],
    #     [0,cos(currentState.chasisState[0]) , -sin(currentState.chasisState[0])],
    #     [0,sin(currentState.chasisState[0]), cos(currentState.chasisState[0])]
    # ]), qb)

    # currentState.chasisState = currentState.chasisState + deltaQ


if __name__ == "__main__":
    initState = youBotState()
    controls = ControlVector()
    controls.wheelSpeeds = np.array([-10,0,-10,0])
    currentState = initState
    totalTime = 1
    timepoints = int(totalTime/youBotProperties.deltaT)
    time = np.linspace(0,totalTime,timepoints)
    for i in time:
        getnextState(currentState,controls)

    print(currentState.chasisState)
