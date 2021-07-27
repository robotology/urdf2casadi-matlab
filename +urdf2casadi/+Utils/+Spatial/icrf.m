function mat = icrf(f)
%Computes the spatial force x velocity cross product taking as input the
%force: icrf(f)*v.

% Import skew function
import urdf2casadi.Utils.Spatial.skew

mat = [-skew(f(1:3)) -skew(f(4:6));-skew(f(4:6)) zeros(3)];
 
