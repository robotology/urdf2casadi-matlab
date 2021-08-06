function R = fromRotationAxisToRotationMatrix(a,q)
%Compute rotation matrix from rotation axis using Rodriguez formula 
% a,q ---> R

% Import skew function 
import urdf2casadi.Utils.Spatial.skew

if all(size(a)==[1,3])
    a = a.';
end
if all(size(a)~= [3,1])
    error('fromRotationAxisToRotationMatrix: input must be a 3D vector');
end
% Consider the case of not normalized rotation axis (norm(a)~=1)
mag = norm(a);
R = eye(3) + sin(mag*q)*skew(a)/mag + (1-cos(mag*q))*(skew(a)^2)/(mag^2);
end