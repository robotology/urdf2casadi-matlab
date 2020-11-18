function [linkName, jointName] = setLinkAndJointName(model, jointIndex)
%Extract from model the name of the link and joint at jointIndex

% The first link is the base link,which for now is excluded from the dynamics
if isfield(model.robot.link{jointIndex+1}.Attributes, 'name')
    linkName = model.robot.link{jointIndex+1}.Attributes.name;
else
    linkName = sprintf('Link %d',jointIndex);
end
if isfield(model.robot.joint{jointIndex}.Attributes, 'name')
    jointName = model.robot.joint{jointIndex}.Attributes.name;
else
    jointName = sprintf('Joint %d',jointIndex);
end
end