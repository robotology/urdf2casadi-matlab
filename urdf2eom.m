function [eq] = urdf2eom(file)
%Generates equation of motion in symbolic form from urdf file 
%Based on code by Roy Featherstone, 2015
%http://royfeatherstone.org/spatial/v2/index.html

choice = input('Enter 1 for FD eq. and 2 for ID eq. -> ');
if choice == 1
    eq = urdf2eomFD(file);
elseif choice == 2
    eq = urdf2eomID(file);
end
end
