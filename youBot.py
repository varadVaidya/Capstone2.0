import numpy as np
import modern_robotics as mr
from math import sin,cos

class youBotProperties:
    """
    contains all the properties for the youBot simulation...
    includes dimensions , Transformaion Matrices , initial configuration and other stuff
    """
    
    robotDim = [0.47/2,0.30/2,0.0475] # Correspomds to length , width of the robot from centre and the radius of the wheels.


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

class TrasformMatrix:
    """
    Contains all the transformation Matrices required for the use in the entire simulation
    and the functions required to do form them..
    """

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


