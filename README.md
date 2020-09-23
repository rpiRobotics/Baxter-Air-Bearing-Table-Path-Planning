# Baxter-Air-Bearing-Table-Path-Planning
Path planning algorithm utilizing resolved velocity motion controller. Application example is planning paths for air bearing table.
Equality/inequality constraints implemented to maintain end effector height and orientation over air bearing plane.

These files are dependent on the matlab-rigid-body-viz package: https://github.com/rpiRobotics/matlab-rigid-body-viz and the general-robotics-toolbox package: https://github.com/rpiRobotics/general-robotics-toolbox Both packages should be added to the current path in Matlab while running. The required subfunctions must also be added to the path. The newest version of Matlab/Simulink may be required to run this simulation.

baxter_pathgen_example.m: basic path planning example for air bearing table

baxter_pathgen_curv_constraint_example.m: path planning example with curvature constraint

baxter_pathgen_curv_constraint_lookahead_example.m: path planning example with curvature constraint and look-ahead term
