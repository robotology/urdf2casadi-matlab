function [smds,model] = extractSystemModel(file)
%Converts URDF file to System Model Data Structure
%as specified in http://royfeatherstone.org/spatial/v2/sysmodel.html
% Import all auxiliary functions
import urdf2casadi.Utils.modelExtractionFunctions.*
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

    %% Store link names
    [linkName, jointName] = setLinkAndJointName(model, i);
    smds.linkName{i} = linkName;
    smds.jointName{i} = jointName;     
end
% Store also inertia of the base link
[I0,m0,com0,rpy0] = computeInertiaWrtCenterOfMass(model, 0);
smds.I_base = I0;
smds.mass_base = m0;
smds.com_base = com0;
smds.rpy_base = rpy0;

% Store base link name
[baseLinkName, ~] = setLinkAndJointName(model, 0);
smds.baseLinkName = baseLinkName;
%% Account for fixed joints
smds = accountForFixedJoint(smds);
end