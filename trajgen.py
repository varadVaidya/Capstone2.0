import math
from math import pi
from youBot import TrasformMatrix, cubePosition, endEffectorinSpace, youBotProperties, youBotState
import numpy as np
import modern_robotics as mr


def solveWaypoints(waypoints,gripper,time):
    """
    takes in waypoints in the form of transformation matrix and gripper state 
    returns flat trajectory in the form of
    r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state
    """
    finalTraj = []
    
    def flatTraj(Trajectory,gripperstate):
        """
        converts the list of transformation matrices into a flatned output and add gripper state
        to convert it to the format required for the V-REP csv scene.
        
        """
        gripperstate = np.array([gripperstate])
        
        # temp1 = Trajectory[0:3,0:3].reshape(9)
        # temp2 = Trajectory[0:3,3]
        traj = []
        for i in range(len(Trajectory)):
            Transformation = np.array(Trajectory[i])
            traj.append(np.concatenate(( Transformation[0:3,0:3].reshape(9), Transformation[0:3,-1] , gripperstate  )))
        
        traj = np.array(traj)
        return traj

    for i in range( len(waypoints) -1 ):

        # traj segment is contains all the Transformation matrix from waypoint i to waypoint i+1 
        trajSegment = flatTraj( mr.ScrewTrajectory(waypoints[i],waypoints[i+1],time[i],time[i]/0.01,method=3), gripper[i] )
        finalTraj.extend(trajSegment)
        

    finalTraj = np.array(finalTraj)
    print(finalTraj.shape)
    return finalTraj

def generateTrajectory(initBotState,initCubeLoc,finalCubeLoc):

    """
    input:
    initBotState:   inital youBot state.
    initCubeLoc :   initial location of cube given by x,y,phi
    finalCubeLoc:   final location of cube given by x,y,phi
    """
    botPosition = endEffectorinSpace(initBotState)
    initCube = cubePosition(initCubeLoc) ## T_sc init
    finalCube = cubePosition(finalCubeLoc) ## T_sc final
    print("init Cube is:",initCube)
    initStandoff = np.dot(initCube,TrasformMatrix.Standoff_Tce)
    print("initStandoff",initStandoff)

    finalStandoff = np.dot(finalCube,TrasformMatrix.Standoff_Tce)

    initGrasp = np.dot(initCube,TrasformMatrix.Grasp_Tce)
    print("initGrasp",initGrasp)
    finalGrasp = np.dot(finalCube,TrasformMatrix.Grasp_Tce)

    waypoints = [botPosition,initStandoff,initGrasp,initGrasp,initStandoff,finalStandoff,finalGrasp,finalGrasp,finalStandoff]
    gripperWaypoint = [0,0,1,1,1,1,0,0]
    time =            [5,3,1,3,5,3,1,3]

    return solveWaypoints(waypoints,gripperWaypoint,time)

if __name__ == "__main__":

    initBotState = youBotState()
    initCube = [1,0,0]
    finalCube = [0,-1,-pi/2]

    trajectory = generateTrajectory(initBotState,initCube,finalCube)
    np.savetxt('traj.csv',trajectory,fmt="%f",delimiter=",")


