# urdf2casadi-matlab
The main purpose of this project is to generate symbolic represantions of the kinematics and dynamics of a robot
by extracting its geometrical and physical parameters from its [URDF](http://wiki.ros.org/urdf) description. It supports fixed-base, open-chain robots.
It uses [CasADi](https://web.casadi.org/) to compute the symbolic expressions. It is inspired by 
[urdf2eom](https://github.com/DeepakParamkusam/urdf2eom). 

## Installation
The software has the following dependencies:
* [MATLAB](https://www.mathworks.com/products/matlab.html);
* [CasADi](https://web.casadi.org/): [Here](https://web.casadi.org/get/) you can find the instruction to install it. Remember to add CasAdi to the MATLAB path.


## Additional features
[iDynTree](https://github.com/robotology/idyntree) has been used to validate the results of the symbolic models.
Its documentation can be found at https://robotology.github.io/idyntree/master/.
Make sure to compile the [bindings to MATLAB](https://github.com/robotology/idyntree#bindings).
Some usefull turorial can be found at https://github.com/robotology/idyntree#tutorials .

## URDF specifications
The URDF specifications and its mathematical description that has been used in this repository can be found [here](https://github.com/robotology/blender-robotics-utils/issues/3#issuecomment-906262419). Only joints with the [`axis`](http://wiki.ros.org/urdf/XML/joint) aligned with one of the three directions of the joint frame are supported. **Models with fixed joints are not supported**.

## Usage 
Add to the [MATLAB path](https://mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html) the repository and all its subfolders by launching `setPath.m`, make sure to insert the correct path to CasADi.Get the [URDF](http://wiki.ros.org/urdf) of your robot. Then pick one of the functions to create and test the model against IDynTree 
in the Verification/ subfolder. Each test* script should be launched from the folder it is contained in.

The code is structured as a MATLAB package. See the [MATLAB documentation](https://it.mathworks.com/help/matlab/matlab_oop/scoping-classes-with-packages.html) on how to reference the package members from within and outside the package.
## Available algorithms
The algorithms generate both symbolic functions and their C code version (that is compiled as a *.mex file):
* [Inverse Dynamics](Dynamics/symbolicInverseDynamics.m);
* [Forward Dynamics](Dynamics/symbolicForwardDynamics.m);
* Computation of the different elements of the Dynamics and Kinematics:
  * [Mass matrix, its derivative and the Coriolis mastrix](Dynamics/createMassAndCoriolisMatrixFunction.m)
  * [Gravity torques ](Dynamics/auxiliarySymbolicDynamicsFunctions/computeGravityTorque.m)
  * [Jacobian and other usefull transforms](Dynamics/auxiliarySymbolicDynamicsFunctions/createSpatialTransformsFunction.m)
* Computation of the [Dynamics of the robot in the form linear in the inertial parameters](Identification/auxiliarySymbolicDynamicsFunctions/computeSymbolicRegressor.m).
   This can be used for identification computing the [stack of the Regressors](Identification/computeSymbolicStackOfRegressors.m). 
   Also the [transposed](Identification/computeSymbolicStackOfRegressorsTransposed.m) of this last function is provided since it requires less computational time to compute the transposed of the stack of regressor matrices. 

 Additionally, the algorithms can be used to simulate a Momentum Observer for external force estimation, which can be tested by:
 * running the script [testMomentumObserver.m](Verfication/testOnMatlab/testMomentumObserver.m) to prepare necessary functions and variables;
 * running the Observer on Simulink using [momentumObserver.slx](Verfication/testOnSimulink);
