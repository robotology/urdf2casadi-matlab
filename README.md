# urdf2eom
Generates symbolic equations of motion from URDF file in MATLAB.

Forward dynamics (using articulated-body algorithm) and inverse dynamics (using recursive Newton-Euler algorithm) supported. Currently supports revolute/continuous joints and fixed-base robots only.

Required tags in URDF file (for each joint or link) - 
[1] Joint axis (along X, Y or Z) 
[2] Link origin 
[3] Joint origin

How to use - Call urdf2eom with the URDF file as the argument. The URDF file can be placed in the URDFs folder. The equations (tau for ID and qdd for FD) are written to text file. 

Code by Deepak Paramkusam, TU Delft. Based on Roy Featherstone's Rigid Body Dynamics Algorithms.
