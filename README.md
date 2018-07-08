# Robotics-Trajectory-Planning-Hybrid-A-Star
Python codes for robotics Trajectory-Planning-Hybrid-A-Star algorithm.
## What is this?
This is a Hybrid-A-Star-Trajectory-Planner. <br>
This method can do the Trajectory-Planning based on Multi-Vehicle-Movement-Sequence-Planning.<br>
The trajectory can be used for non-holonomic vehicle.<br>
# Cooperative Trajectory Planning
We have obtained the planning results based on the topological map.<br>
Now we want to do trajectory planning based on grid-based map.<br>
<img src="https://github.com/ChenBohan/Robotics-Trajectory-Planning-Hybrid-A-Star/blob/master/pic/multi-vehicle%20trajectory.png" width = "50%" height = "50%" div align=center />

## Trajectory Generator 
Hybrid A* Planner<br>
Graph：State Lattice + Grid Map<br>
Trajectory：Reeds Shepp Path Generator（non-holonomic）<br>
Algorithm：Based on A*<br>
Input：1. Start state；2. Goal state；3. Grid-based Map；
Output：Trajectory
<img src="https://github.com/ChenBohan/Robotics-Trajectory-Planning-Hybrid-A-Star/blob/master/pic/planner.png" width = "50%" height = "50%" div align=center />

## State Lattice
State Lattice uses the model predictive trajectory generator to solve boundary problem.<br>
<img src="https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/ModelPredictiveTrajectoryGenerator/lookuptable.png?raw=True" width = "50%" height = "50%" div align=center />

Ref:<br>
[Optimal rough terrain trajectory generation for wheeled mobile robots](https://www.ri.cmu.edu/pub_files/pub1/thrun_sebastian_1996_1/thrun_sebastian_1996_1.pdf "Learning metric-topological maps for indoor mobile robot navigation")
[State Space Sampling of Feasible Motions for High-Performance Mobile Robot Navigation in Complex Environments](http://www.frc.ri.cmu.edu/~alonzo/pubs/papers/JFR_08_SS_Sampling.pdf)

## Reeds Shepp Path
Ref:<br>
[Practical Search Techniques in Path Planning for Autonomous Driving](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)






