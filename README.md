# Design Study of 6DOF Robotic Arm for Additive Manufacturing 
This design study uses the inverse kinematics equations of a general PUMA type robot arm to find the optimal link lengths for FDM 3D printing. The user specifies a build volume and the MATLAB scripts iteratively computes the reachabillity of the arm while ensuring that the arm will not collide with printbed and current print. Using numerical optimization we assign a cost to each set of link lengths using a user defined cost function and return the optimal link lengths that allow it reach the entire build volume without collision. Using this technique the script also returns the optimal position of the robots base relative to the print volume.

![Image of maniputlator](https://github.com/michaelmenaker/6DOF_Optimization/blob/master/images/4R_manip.jpg)

