function X_Ti = computeBodyJointTransforms(model, jointIndex)
%Compute X_Ti:
% relative transform from teh local frame of the body parent(i)(F_{i}) to the location of the frame of
% joint i wrt parent of link i(F_{pi,i}).
% Look at the RBDL implementation for comparison:
% https://github.com/rbdl/rbdl/blob/bf42f2552bbb6c872bc275a608762b0f91aa2dd3/addons/urdfreader/urdfreader.cc#L215

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
X_Ti = rotx(rpy(1))*roty(rpy(2))*rotz(rpy(3))*xlt(t);
end