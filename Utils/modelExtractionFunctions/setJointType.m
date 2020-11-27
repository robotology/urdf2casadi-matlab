function [type,axis] = setJointType(model, jointIndex)
%Based on the axis of the joint it determines the joint type
% Look at http://wiki.ros.org/urdf/XML/joint for all possible choises

% Set default value for the type and axis
type = '';
axis = [0 0 0];
if (strcmp(model.robot.joint{1,jointIndex}.Attributes.type,'fixed'))   
    if  isfield(model.robot.joint{1,jointIndex}, 'axis')
        axis = str2num(model.robot.joint{1,jointIndex}.axis.Attributes.xyz);
    else
        axis = [0 0 0];
    end
    type = 'fixed';
end

% Revolute with or without limits
if (strcmp(model.robot.joint{1,jointIndex}.Attributes.type,'continuous') || strcmp(model.robot.joint{1,jointIndex}.Attributes.type,'revolute'))    
    if  isfield(model.robot.joint{1,jointIndex}, 'axis')
        axis = str2num(model.robot.joint{1,jointIndex}.axis.Attributes.xyz);
    else
        warning('Axis field missing for joint %d', jointIndex);
        % Default axis value according to http://wiki.ros.org/urdf/XML/joint
        axis = [1 0 0];
    end
    if ((axis(1)~=0) && (axis(2)==0) && (axis(3)==0))
        type = 'Rx';
    end
    if ((axis(1)==0) && (axis(2)~=0) && (axis(3)==0))
        type = 'Ry';
    end
    if ((axis(1)==0) && (axis(2)==0) && (axis(3)~=0))
        type = 'Rz';
    end
end
% Prismatic
if (strcmp(model.robot.joint{1,jointIndex}.Attributes.type,'prismatic'))
    if  isfield(model.robot.joint{1,jointIndex}, 'axis')
        axis = str2num(model.robot.joint{1,jointIndex}.axis.Attributes.xyz);
    else
        % Default axis value according to http://wiki.ros.org/urdf/XML/joint
        axis = [1 0 0];
    end
    if ((axis(1)~=0) && (axis(2)==0) && (axis(3)==0))
        type = 'Px';
    end
    if ((axis(1)==0) && (axis(2)~=0) && (axis(3)==0))
        type = 'Py';
    end
    if ((axis(1)==0) && (axis(2)==0) && (axis(3)~=0))
        type = 'Pz';
    end
end
end