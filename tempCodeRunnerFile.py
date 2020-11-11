erate Trajectory ########
Trajectory = generateTrajectory(initialState,initCube,finalCube)
np.savetxt('traj.csv',Trajectory,fmt="%f",delimiter=",")
# trajectory defined