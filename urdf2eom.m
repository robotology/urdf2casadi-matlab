function [model] = urdf2eom(file)
addpath('xml2struct')
addpath('URDFs')

% Load URDF
model = xml2struct(file);

% Detect no. of links and joints
n_links = size(model.robot.link);
n_joints = size(model.robot.joint);

% Initialize variables
sym('m',n_links);
sym('I',n_links);
sym('q',n_joints);
sym('dq',n_joints);
sym('ddq',n_joints);
syms g 







end