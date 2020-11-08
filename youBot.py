import numpy as np
import modern_robotics as mr
from math import sin,cos

from numpy.lib.function_base import _select_dispatcher

class youBotProperties:
    """
    contains all the properties for the youBot simulation...
    includes dimensions , Transformaion Matrices , initial configuration and other stuff
    """
    def __init__(self):
        pass
    robotDim = [0.47/2,0.30/2,0.0475] # Correspomds to length , width of the robot from centre and the radius of the wheels.
    l,w,r = robotDim

    Blist = np.array([
            [0,0,1,0,0.033,0],
            [0,-1,0,-0.5076,0,0],
            [0,-1,0,-0.3526,0,0],
            [0,-1,0,-0.2176,0,0],
            [0,0,1,0,0,0]
        ]).T

    homePositionM0e = np.array([
        [1,0,0,0.033],
        [0,1,0,0],
        [0,0,1,0.6546],
        [0,0,0,1]
        ])

    F_theta = (r/4) * np.array([
        [-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)],
        [1       ,1      ,1      ,1       ],
        [-1      ,1      ,-1     ,1]
    ])

    jointSpeedLimit = 12.9
    wheelSpeedLimit = 12.9
    deltaT = 0.01
    
    @staticmethod
    def saturate(variable,limit):
        """
        saturates variable within 
        -limit <= variable <= limit    
        """
        variable = np.clip(variable,a_min= -1 * limit, a_max= limit)
        return variable

class youBotState:
    def __init__(self):
        self.chasisState = np.zeros(3)
        self.jointState = np.zeros(5)
        self.wheelState = np.zeros(4)
        self.gripperState = np.zeros(1)

class ControlVector:
    def __init__(self):
        self.jointSpeeds = np.zeros(5)
        self.wheelSpeeds = np.zeros(4)
        self.griperControl = np.zeros(1) # could be redundant....


class TrasformMatrix:
    """
    Contains all the transformation Matrices required for the use in the entire simulation
    and the functions required to do form them..
    """
    T_b0  = np.array([
        [1,0,0,0.1662],
        [0,1,0,0],
        [0,0,1,0.0026],
        [0,0,0,1]
    ])
    
    Standoff_Tce = np.array([
        [-1,0,0,0],
        [0,1,0,0],
        [0,0,-1,0.2],
        [0,0,0,1]
    ])
    
    Grasp_Tce = np.array([
        [-1,0,0,0],
        [0,1,0,0],
        [0,0,-1,0],
        [0,0,0,1]
    ])

    def cubePosition(self,x,y,phi):
        """
        give the position of the cube in the space frame {s}

        used for both initial and final configurations
        """
        return np.array([
            [cos(phi),-sin(phi),0,x],
            [sin(phi),cos(phi),y],
            [0,0,1,0.25],
            [0,0,0,1]
        ])

    def basePosition(self,chasisConfig):
        """
        input: takes in Chasis configuration of the robot 
        Chasis Config represented by phi,x,y
        returns: the transformation matrix of the base in the space frame {s}        
        """
        phi,x,y = chasisConfig

        return np.array([
            [cos(phi),-sin(phi),0,x],
            [sin(phi),cos(phi),y],
            [0,0,1,0.0963],
            [0,0,0,1]
        ])

    def endEffectorinSpace(self,currentState):
        """
        input:
        current state
        returns:
        the transformation matrix of the end effector in space frame.
        """
        base = self.basePosition(currentState.chasisState)
        offset = self.T_b0
        forwardkin = mr.FKinBody(youBotProperties.homePositionM0e,youBotProperties.Blist,currentState.jointState)

        return np.dot(base,offset,forwardkin)
    
    


