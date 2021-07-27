function lambda = computeParentList(model)
%Computes for each link its parent:
% Only kinematic chains are supported for now
% Number of joints
numJoints = size(model.robot.joint,2);
numLinks = numJoints+1;

% Look for the parent link for each body expect the base link
for i = 1:numJoints
    for j = 1:numLinks
        if strcmp(model.robot.joint{i}.parent.Attributes.link, model.robot.link{j}.Attributes.name)
            lambda(i) = j-1;
        end
    end
end