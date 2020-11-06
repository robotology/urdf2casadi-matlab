function [smds,model] = extractSystemModel(file)
%Converts URDF file to System Model Data Structure
%as specified in http://royfeatherstone.org/spatial/v2/sysmodel.html
%Load URDF
model = xml2struct(file);
% model = importrobot(file);
% model = URDF(file);

%NB: total number of joints
smds.NB = size(model.robot.joint,2);
if smds.NB == 1
    model.robot.joint = num2cell(model.robot.joint);
end
% Extract inertia info from urdf or use symbolic values also for the
% inertia parameters
use_urdf_inertia_param = true;
smds.use_urdf_inertia_param = use_urdf_inertia_param;
if use_urdf_inertia_param
    I = zeros(3,3);
    m = zeros(smds.NB+1,1);
    smds.I = {};
else
    import casadi.*;
    I = casadi.SX.sym('I',3,3,smds.NB);
    m = casadi.SX.sym('m',1,smds.NB);
    smds.I = casadi.SX.sym('I',6,6,smds.NB);
end

for i = 1:smds.NB
    %% Compute links parent
    smds.parent = computeParentList(model);
    %% Set joint type
    % It depends on the type of joint (in particular its axis): look at
    % http://wiki.ros.org/urdf/XML/joint for all possible choises
    [type,axis] = setJointType(model, i);
    smds.jtype{i} = type;
    smds.jaxis{i} = axis;
    %% Compute Transform Tree:
    % Transform from parent(i) local frame 
    % to the frame representing the relative position of the frame of joint i wrt the frame of parent(i)
    smds.Xtree{i} = computeBodyJointTransforms(model,i);    
    %% Compute inertia
    [I,m,com,rpy] = computeInertiaWrtCenterOfMass(model, i);
    smds.I{1,i} = I;
    smds.mass{1,i} = m;
    smds.com{1,i} = com.';
    smds.rpy{1,i} = rpy.';
end
%% Accorporate bodies connected by `fixed`  joints 
% Find indeces of fixed joints
fixedJoints = find(ismember(smds.jtype,'fixed'));

if ~isempty(fixedJoints)
    % Remove fixed joints from list of joint types and axis
    smds.jtype(fixedJoints) = [];
    smds.jaxis(fixedJoints) = [];

    for j = 1:max(size(fixedJoints))

        % Update the transform considering that the joint is fixed 
        % Exclude the last joint as it connects with the end effector and no
        % link down this subtree will feel the presence of the fixed joint, if not only fo the inertia(which must be summed)
        if fixedJoints(end) < smds.NB
            smds.Xtree{fixedJoints(j)+1} = smds.Xtree{fixedJoints(j)+1}*smds.Xtree{fixedJoints(j)};
        end
        % Update parents list
        if  fixedJoints(1)>=1 && fixedJoints(end)< smds.NB
            smds.parent(fixedJoints(j)+1) = fixedJoints(j)-1;
        end
        % Delete from the list of parents the entry of the body with index
        % fixedJoint(j), even if it connects with the base link
        smds.parent(fixedJoints(j)) = [];

        % Update spatial inertia matrix of bodies connected by fixed joints
        % Sum the spatial inertia of the body connected by the fixed joint by
        % first expressing the inestia in the same reference frame
        % Exclude the first joint from the count:
        % If the joint connecting the first link with the base link is fixed
        % its inertia is added to the base link, which for now is condered
        % fixed.
        if fixedJoints(1)~=1
            smds.I{1,fixedJoints(j)} = smds.I{1,fixedJoints(j)-1} + (smds.Xtree{fixedJoints(j)}.')*smds.I{1,fixedJoints(j)-1}*smds.Xtree{fixedJoints(j)};
        end
        % Delete body at fixedJoints(j) index from body list
        smds.I(fixedJoints(j)) = [];
        % Delete fixed joint from joint list
        smds.Xtree(fixedJoints(j)) = [];

    end
    % Update the final number of considered joints
    smds.NB = smds.NB - max(size(fixedJoints));
end
end