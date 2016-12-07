# Verification using RBDL

This folder contains files for verifying the results obtained from urdf2eom. RBDL library has been used for verification.

Prerequistes before running codes -
1. RBDl (make urdf reader also)  
2. Eigen  
3. MATLAB

Make the code in eval_test2L_rbdl and compare its results against those obtained by running eval_test2L.m. For test2L.urdf using FD (upto 4 decimal places) - 
qdd1 = -0.0936
qdd2 = 0.4499

Can be tested for other urdfs and ID too.

- Deepak Paramkusam
