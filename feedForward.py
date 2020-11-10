from scipy.linalg import pinv2
from numpy.lib import utils
from youBot import ControlVector, basePosition, endEffectorinSpace, youBotProperties, youBotState
import numpy as np
import modern_robotics as mr

def feedForwardControl(X,Xd,XdNext,currentState,controls,Kp,Ki):
    """
    sets the controVector to be passed on to the next state function.
    """
    T_eb = np.dot( np.linalg.inv(endEffectorinSpace(currentState)), basePosition(currentState.chasisState))
    
    Jbase = np.dot(mr.Adjoint(T_eb),youBotProperties.F6)
    Jarm = mr.JacobianBody(youBotProperties.Blist,currentState.jointState)

    Je = np.concatenate((Jbase,Jarm),axis=1)
    # Je = np.concatenate((Jarm,Jbase),axis=1)
    invJe = pinv2(Je)
    VdBracket = (1/youBotProperties.deltaT) * mr.MatrixLog6(np.dot(np.linalg.inv(Xd),XdNext))
    Vd = mr.se3ToVec(VdBracket)
    #print("Vd",Vd)

    Xerr = mr.se3ToVec( mr.MatrixLog6(np.dot(np.linalg.inv(X),Xd)) )
    youBotProperties.ErrorInt = youBotProperties.ErrorInt + Xerr * youBotProperties.deltaT
    print(Xerr)

    feedForward = np.dot(mr.Adjoint(np.dot(np.linalg.inv(X),Xd)),Vd)
    #print("FeedForward:",feedForward)
    V = feedForward + np.dot(Kp,Xerr) * np.dot(Ki,youBotProperties.ErrorInt)
    #print("V",V)

    u_thetadot = np.dot(invJe,V)
    controls.wheelSpeeds = u_thetadot[0:4]
    controls.jointSpeeds = u_thetadot[4:9]
    # controls.wheelSpeeds = u_thetadot[5:9]
    # controls.jointSpeeds = u_thetadot[0:5]
    # print("wheel COntrols",u_thetadot[0:4]
    # )
    # print("Joint COntrols",u_thetadot[4:9])
    return Xerr





if __name__ == "__main__":

    Xd = np.array([
        [0,0,1,0.5],
        [0,1,0,0],
        [-1,0,0,0.5],
        [0,0,0,1]
    ])
    XdNext = np.array([
        [0,0,1,0.6],
        [0,1,0,0],
        [-1,0,0,0.3],
        [0,0,0,1]
    ])

    currentState = youBotState()
    controls = ControlVector()
    currentState.jointState = np.array([0,0,0.2,-1.6,0])
    X = endEffectorinSpace(currentState)
    #print("X",X)
    Kp =  0 * np.eye(6)
    feedForwardControl(X,Xd,XdNext,currentState,controls,Kp)
    #print("Joint Speeds",controls.jointSpeeds)
    pass