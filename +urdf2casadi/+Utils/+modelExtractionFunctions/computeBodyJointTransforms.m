function X_Ti = computeBodyJointTransforms(model,jointIndex)
%Compute X_Ti:
% relative transform from teh local frame of the body parent(i)(F_{i}) to the location of the frame of
% joint i wrt parent of link i(F_{pi,i}).
% Look at the RBDL implementation for comparison:
% https://github.com/rbdl/rbdl/blob/bf42f2552bbb6c872bc275a608762b0f91aa2dd3/addons/urdfreader/urdfreader.cc#L215

% Import spatial functions 
import urdf2casadi.Utils.Spatial.*

if isfield(model.robot.joint{1,jointIndex}.origin.Attributes,'xyz')
    t = str2num(model.robot.joint{1,jointIndex}.origin.Attributes.xyz);
else
    error('Joint %d missing frame origin position', jointIndex);
end
if isfield(model.robot.joint{1,jointIndex}.origin.Attributes,'rpy')
    rpy = str2num(model.robot.joint{1,jointIndex}.origin.Attributes.rpy);
else
    error('Joint %d missing frame orientation', jointIndex);
end

%% In the urdf convention, given a rotarion from frame B to A
% first rotate about x of the frame B of roll radiants
% second rotate about y of the frame B of pitch radiants
% third rotate about z of the frame B of yaw radiants;
% Look at page 85 (Ch. 4.5) Featherstone, Rigid Body Dynnamis algorithms(2007, Sringer)
X_Ti = rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1))*xlt(t);
end