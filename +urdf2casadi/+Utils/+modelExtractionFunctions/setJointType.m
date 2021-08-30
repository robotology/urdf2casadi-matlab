function [type,axis] = setJointType(model, jointIndex)
%Based on the axis of the joint it determines the joint type
% Look at http://wiki.ros.org/urdf/XML/joint for all possible choises

% Set default value for the type and axis
type = '';
axis = [0 0 0];
zeroTol = 1e-5;
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
    % Check the axis vector components. Take into account possible numerical errors in the URDFs with non
    % perfectly null `axis` vector components using a small threshold.
    if (abs(axis(1))>= 1-zeroTol && abs(axis(2))<=zeroTol && abs(axis(3))<=zeroTol)
        type = 'Rx';
    end
    if (abs(axis(1))<=zeroTol && abs(axis(2))>= 1-zeroTol && abs(axis(3))<=zeroTol)
        type = 'Ry';
    end
    if (abs(axis(1))<=zeroTol && abs(axis(2))<=zeroTol && abs(axis(3))>= 1-zeroTol)
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
    % Check the axis vector components. Take into account possible numerical errors in the URDFs with non
    % perfectly null `axis` vector components using a small threshold.
    if (abs(axis(1))>= 1-zeroTol && abs(axis(2))<=zeroTol && abs(axis(3))<=zeroTol)
        type = 'Px';
    end
    if (abs(axis(1))<=zeroTol && abs(axis(2))>= 1-zeroTol && abs(axis(3))<=zeroTol)
        type = 'Py';
    end
    if (abs(axis(1))<=zeroTol && abs(axis(2))<=zeroTol && abs(axis(3))>= 1-zeroTol)
        type = 'Pz';
    end
end
end