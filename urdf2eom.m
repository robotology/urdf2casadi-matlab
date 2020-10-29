function [eq] = urdf2eom(file, generate_c_code)
%Generates equations of motion in symbolic form from urdf file 
%Based on Rigid Body Dynamics Algorithms by Roy Featherstone
%http://royfeatherstone.org/spatial/v2/index.html

%% Add casadi to Matlab path
addpath('/home/iiticublap041/casadi-linux-matlabR2014b-v3.5.3');

choice = input('Enter 1 for FD eq. and 2 for ID eq. -> ');
if choice == 1
    eq = urdf2eomFD(file,generate_c_code);
elseif choice == 2
    eq = urdf2eomID(file,generate_c_code);
end
end
