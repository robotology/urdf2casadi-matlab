function [smds,model] = urdf2smds(file)
%Converts URDF file to System Model Data Structure
%as specified in http://royfeatherstone.org/spatial/v2/sysmodel.html

addpath('xml2struct')
addpath('URDFs')
addpath('Spatial')

%Load URDF
model = xml2struct(file);

%NB
smds.NB = size(model.robot.joint,2);
if smds.NB == 1
    model.robot.joint = num2cell(model.robot.joint);
end

%Initilizations
smds.parent = zeros(1,smds.NB);
smds.jtype = {};
% sym_I = sym('I%d',[3,3,smds.NB],'real');
% sym_m = sym('m',[1,smds.NB],'real');

import casadi.*;
% Extract inertia info from urdf or use symbolic values also for the
% inertia parameters
use_urdf_inertia_param = true;

if ~use_urdf_inertia_param
    sym_I = SX.sym('I%d',3,3,smds.NB);
    sym_m = SX.sym('m',1,smds.NB);
    smds.I = SX.sym('I%d',6,6,smds.NB);
else
    I = zeros(3,3,smds.NB+1);
    m = zeros(smds.NB+1,1);
    smds.I = zeros(6,6,smds.NB);
end

%Generation loop
for i = 1:smds.NB
    %Parents
    for j = 1:smds.NB+1
        if strcmp(model.robot.joint{i}.parent.Attributes.link,model.robot.link{j}.Attributes.name)
            smds.parent(i) = j-1;
        end
    end
    
    %jType
    if strcmp(model.robot.joint{i}.Attributes.type,'revolute')|| strcmp(model.robot.joint{i}.Attributes.type,'continuous')
        if strcmp(model.robot.joint{i}.axis.Attributes.xyz,'1 0 0') ||strcmp(model.robot.joint{i}.axis.Attributes.xyz,'-1 0 0')
            smds.jtype{i} = 'Rx';
        elseif strcmp(model.robot.joint{i}.axis.Attributes.xyz,'0 1 0') ||strcmp(model.robot.joint{i}.axis.Attributes.xyz,'0 -1 0')
            smds.jtype{i} = 'Ry';
        elseif strcmp(model.robot.joint{i}.axis.Attributes.xyz,'0 0 1') ||strcmp(model.robot.joint{i}.axis.Attributes.xyz,'0 0 -1')
            smds.jtype{i} = 'Rz';
        end
    end
    
    %Inertia
    com = str2num(model.robot.link{i+1}.inertial.origin.Attributes.xyz);

    if ~use_urdf_inertia_param
        smds.I{1,i} = mcI(sym_m(i),com,sym_I{1,i});
    else
        m(i)      = str2num(model.robot.link{i+1}.inertial.mass.Attributes.value);
        I(1,1,i) = str2num(model.robot.link{i+1}.inertial.inertia.Attributes.ixx);
        I(1,2,i) = str2num(model.robot.link{i+1}.inertial.inertia.Attributes.ixy);
        I(1,3,i) = str2num(model.robot.link{i+1}.inertial.inertia.Attributes.ixz);
        I(2,1,i) = str2num(model.robot.link{i+1}.inertial.inertia.Attributes.ixy);
        I(2,2,i) = str2num(model.robot.link{i+1}.inertial.inertia.Attributes.iyy);
        I(2,3,i) = str2num(model.robot.link{i+1}.inertial.inertia.Attributes.iyz);
        I(3,1,i) = str2num(model.robot.link{i+1}.inertial.inertia.Attributes.ixz);
        I(3,2,i) = str2num(model.robot.link{i+1}.inertial.inertia.Attributes.iyz);
        I(3,3,i) = str2num(model.robot.link{i+1}.inertial.inertia.Attributes.izz);
        smds.I(:,:,i) = mcI(m(i),com,I(:,:,i));
    end
    %Transform tree
    if smds.parent(i) == 0
        angle = str2num(model.robot.joint{i}.origin.Attributes.rpy);
        disp = str2num(model.robot.joint{i}.origin.Attributes.xyz);
    else
        angle = str2num(model.robot.joint{i}.origin.Attributes.rpy)-str2num(model.robot.joint{i-1}.origin.Attributes.rpy);
        disp = str2num(model.robot.joint{i}.origin.Attributes.xyz)-str2num(model.robot.joint{i-1}.origin.Attributes.xyz);
    end
    smds.Xtree{i} = rotx(angle(1))*roty(angle(2))*rotz(angle(3)) * xlt(disp);
end
    smds.use_urdf_inertia_param = use_urdf_inertia_param;
end
