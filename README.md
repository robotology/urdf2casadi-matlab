# urdf2casadi-matlab
The main purpose of this project is to generate symbolic represantions of the kinematics and dynamics of a robot
by extracting its geometrical and physical parameters from its [URDF](http://wiki.ros.org/urdf) description.
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

## Usage 
Get the [URDF](http://wiki.ros.org/urdf) of your robot. Then pick one of the functions to create and test the model against IDynTree 
in the Verification/ subfolder. 
