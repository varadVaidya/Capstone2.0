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
    zeroMatrix  = np.zeros(np.shape(F_theta)[1])

    F6 = np.concatenate((zeroMatrix[np.newaxis,:],zeroMatrix[np.newaxis,:],F_theta,zeroMatrix[np.newaxis,:]),axis=0)

    jointSpeedLimit = 1000000000000000
    wheelSpeedLimit = 400000000000000000
    deltaT = 0.01
    ErrorInt = np.zeros(6)
    
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
    def __init__(self):
        pass

    T_b0  = np.array([
        [1,0,0,0.1662],
        [0,1,0,0],
        [0,0,1,0.0026],
        [0,0,0,1]
    ])
    
    # Standoff_Tce = np.array([[-0.17364818,  0.        ,  0.98480775,  0.        ],
    #    [ 0.        ,  1.        ,  0.        ,  0.        ],
    #    [-0.98480775,  0.        , -0.17364818,  0.2        ],
    #    [ 0.        ,  0.        ,  0.        ,  1.        ]])
    
    # Standoff_Tce = np.array([
    #     [-1,0,0,0],
    #     [0,1,0,0],
    #     [0,0,-1,0.2],
    #     [0,0,0,1]
    # ])
    
    
    Standoff_Tce = np.array([[-0.34202014,  0.        ,  0.93969262,  0.05        ],
       [ 0.        ,  1.        ,  0.        ,  0.        ],
       [-0.93969262,  0.        , -0.34202014,  0.2      ],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])



    # Grasp_Tce = np.array([
    #     [0,0,1,0],
    #     [0,1,0,0],
    #     [-1,0,0,0],
    #     [0,0,0,1]
    # ])

    Grasp_Tce = np.array([[-0.34202014,  0.        ,  0.93969262,  0.01        ],
       [ 0.        ,  1.        ,  0.        ,  0.        ],
       [-0.93969262,  0.        , -0.34202014,  0.        ],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
    # Grasp_Tce = np.array([[-0.17364818,  0.        ,  0.98480775,  0.        ],
    #    [ 0.        ,  1.        ,  0.        ,  0.        ],
    #    [-0.98480775,  0.        , -0.17364818,  0.       ],
    #    [ 0.        ,  0.        ,  0.        ,  1.        ]])

    # Grasp_Tce = np.array([
    #     [-1,0,0,0],
    #     [0,1,0,0],
    #     [0,0,-1,0],
    #     [0,0,0,1]
    # ])


def cubePosition(cubePos):
    """
    give the position of the cube in the space frame {s}
    x,y,phi
    used for both initial and final configurations
    """
    x,y,phi = cubePos

    cube = np.array([
        [cos(phi),-sin(phi),0,x],
        [sin(phi),cos(phi),0,y],
        [0,0,1,0.025],
        [0,0,0,1]
    ])
    return cube

def basePosition(chasisConfig):
    """
    input: takes in Chasis configuration of the robot 
    Chasis Config represented by phi,x,y
    returns: the transformation matrix of the base in the space frame {s}        
    """
    phi,x,y = chasisConfig

    base = np.array([
        [cos(phi),-sin(phi),0,x],
        [sin(phi),cos(phi),0,y],
        [0,0,1,0.0963],
        [0,0,0,1]
    ])
    return base
    # return np.array([
    #     [cos(phi),-sin(phi),0,x],
    #     [sin(phi),cos(phi),y],
    #     [0,0,1,0.0963],
    #     [0,0,0,1]
    # ])

def endEffectorinSpace(currentState):
    """
    input:
    current state
    returns:
    the transformation matrix of the end effector in space frame.
    """
    base = basePosition(currentState.chasisState)
    offset = TrasformMatrix.T_b0
    forwardkin = mr.FKinBody(youBotProperties.homePositionM0e,youBotProperties.Blist,currentState.jointState)
    endEffector = np.dot(base,np.dot(offset,forwardkin))
    return endEffector

def flatTrajtoTransform(traj):
    gripper = traj[-1]

    rotation = traj[0:9].reshape(3,3)
    point = traj[9:12]

    transform = mr.RpToTrans(rotation,point)
    return transform,gripper
    


