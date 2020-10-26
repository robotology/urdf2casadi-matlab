function [eq] = urdf2eom(file)
%Generates equations of motion in symbolic form from urdf file 
%Based on Rigid Body Dynamics Algorithms by Roy Featherstone
%http://royfeatherstone.org/spatial/v2/index.html

choice = input('Enter 1 for FD eq. and 2 for ID eq. -> ');
flag = input('Try to geneate and compile c code? (y/n)','s');

if strcmp(flag,'y')
    geneate_c_code = 1;
else
    geneate_c_code = 0;
end

if choice == 1
    eq = urdf2eomFD(file,geneate_c_code);
elseif choice == 2
    eq = urdf2eomID(file,geneate_c_code);
end
end
