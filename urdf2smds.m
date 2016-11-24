<<<<<<< HEAD
function [smds,model] = urdf2smds(file)
% Converts URDF file to System Model Data Structure
% as specified in http://royfeatherstone.org/spatial/v2/sysmodel.html

addpath('xml2struct')
addpath('URDFs')
addpath('Spatial')

% Load URDF
model = xml2struct(file);

% NB
smds.NB = size(model.robot.joint,2);
if smds.NB == 1
    model.robot.joint = num2cell(model.robot.joint);
end

% Initilizations
smds.parent = zeros(1,smds.NB);
smds.jtype = {};
sym_I = sym('I%d',[3,3,smds.NB]);
sym_m = sym('m',[1,smds.NB]);

% Generation loop
for i = 1:smds.NB
    % Parents
    for j = 1:smds.NB+1
        if strcmp(model.robot.joint{i}.parent.Attributes.link,model.robot.link{j}.Attributes.name)
            smds.parent(i) = j-1;
        end
    end
    
    % jType
    if strcmp(model.robot.joint{i}.Attributes.type,'revolute')|| strcmp(model.robot.joint{i}.Attributes.type,'continuous')
        if strcmp(model.robot.joint{i}.axis.Attributes.xyz,'1 0 0') ||strcmp(model.robot.joint{i}.axis.Attributes.xyz,'-1 0 0')
            smds.jtype{i} = 'Rx';
        elseif strcmp(model.robot.joint{i}.axis.Attributes.xyz,'0 1 0') ||strcmp(model.robot.joint{i}.axis.Attributes.xyz,'0 -1 0')
            smds.jtype{i} = 'Ry';
        elseif strcmp(model.robot.joint{i}.axis.Attributes.xyz,'0 0 1') ||strcmp(model.robot.joint{i}.axis.Attributes.xyz,'0 0 -1')
            smds.jtype{i} = 'Rz';
        end
    end
    
    % Inertia
    com = str2num(model.robot.link{i+1}.inertial.origin.Attributes.xyz);
    smds.I(:,:,i) = mcI(sym_m(i),com,sym_I(:,:,i));
    
    % Transform tree
    if smds.parent(i) == 0
        angle = str2num(model.robot.joint{i}.origin.Attributes.rpy);
        disp = str2num(model.robot.joint{i}.origin.Attributes.xyz);
    else
        angle = str2num(model.robot.joint{i}.origin.Attributes.rpy)-str2num(model.robot.joint{i-1}.origin.Attributes.rpy);
        disp = str2num(model.robot.joint{i}.origin.Attributes.xyz)-str2num(model.robot.joint{i-1}.origin.Attributes.xyz);
    end
    smds.Xtree{i} = rotx(angle(1))*roty(angle(2))*rotz(angle(3)) * xlt(disp);
end
end
=======
function [smds,model] = urdf2smds(file)
% Converts URDF file to System Model Data Structure 
% as specified in http://royfeatherstone.org/spatial/v2/sysmodel.html

addpath('xml2struct')
addpath('URDFs')
addpath('Spatial')

% Load URDF
model = xml2struct(file);

% NB and gravity
smds.NB = size(model.robot.joint,2);
%smds.g = [0,0,-9.81];

% Parents
smds.parent = zeros(1,smds.NB);
if smds.NB == 1
    smds.parent(1) = 0;
else
    for i = 1:smds.NB
        for j = 1:smds.NB+1
            if strcmp(model.robot.joint{i}.parent.Attributes.link,model.robot.link{j}.Attributes.name)
                smds.parent(i) = j-1;
            end
        end
    end
end

% jType
smds.jtype = {};
for i = 1:smds.NB
    if smds.NB == 1
        if strcmp(model.robot.joint(i).Attributes.type,'revolute')|| strcmp(model.robot.joint(i).Attributes.type,'continuous')
            if strcmp(model.robot.joint(i).axis.Attributes.xyz,'1 0 0') ||strcmp(model.robot.joint(i).axis.Attributes.xyz,'-1 0 0')
                smds.jtype{i} = 'Rx';
            elseif strcmp(model.robot.joint(i).axis.Attributes.xyz,'0 1 0') ||strcmp(model.robot.joint(i).axis.Attributes.xyz,'0 -1 0')
                smds.jtype{i} = 'Ry';
            elseif strcmp(model.robot.joint(i).axis.Attributes.xyz,'0 0 1') ||strcmp(model.robot.joint(i).axis.Attributes.xyz,'0 0 -1')
                smds.jtype{i} = 'Rz';
            end
        end
    else
        if strcmp(model.robot.joint{i}.Attributes.type,'revolute')|| strcmp(model.robot.joint{i}.Attributes.type,'continuous')
            if strcmp(model.robot.joint{i}.axis.Attributes.xyz,'1 0 0') ||strcmp(model.robot.joint{i}.axis.Attributes.xyz,'-1 0 0')
                smds.jtype{i} = 'Rx';
            elseif strcmp(model.robot.joint{i}.axis.Attributes.xyz,'0 1 0') ||strcmp(model.robot.joint{i}.axis.Attributes.xyz,'0 -1 0')
                smds.jtype{i} = 'Ry';
            elseif strcmp(model.robot.joint{i}.axis.Attributes.xyz,'0 0 1') ||strcmp(model.robot.joint{i}.axis.Attributes.xyz,'0 0 -1')
                smds.jtype{i} = 'Rz';
            end
        end
    end
end


% Inertia
sym_I = sym('I%d',[3,3,smds.NB]);
sym_m = sym('m',[1,smds.NB]);
for i = 1:smds.NB
    com = str2num(model.robot.link{i+1}.inertial.origin.Attributes.xyz);
    smds.I(:,:,i) = mcI(sym_m(i),com,sym_I(:,:,i));
end

% Transform tree
for i = 1:smds.NB    
    if smds.NB == 1
        if smds.parent(i) == 0
            angle = str2num(model.robot.joint(i).origin.Attributes.rpy);
            disp = str2num(model.robot.joint(i).origin.Attributes.xyz);
        else
            angle = str2num(model.robot.joint(i).origin.Attributes.rpy)-str2num(model.robot.joint(i-1).origin.Attributes.rpy);
            disp = str2num(model.robot.joint(i).origin.Attributes.xyz)-str2num(model.robot.joint(i-1).origin.Attributes.xyz);
        end
    else
        if smds.parent(i) == 0
            angle = str2num(model.robot.joint{i}.origin.Attributes.rpy);
            disp = str2num(model.robot.joint{i}.origin.Attributes.xyz);
        else
            angle = str2num(model.robot.joint{i}.origin.Attributes.rpy)-str2num(model.robot.joint{i-1}.origin.Attributes.rpy);
            disp = str2num(model.robot.joint{i}.origin.Attributes.xyz)-str2num(model.robot.joint{i-1}.origin.Attributes.xyz);
        end
    end
    smds.Xtree{i} = rotx(angle(1))*roty(angle(2))*rotz(angle(3)) * xlt(disp);
end
end
>>>>>>> master
