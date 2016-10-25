function [model] = urdf2eom(file)
addpath('xml2struct')
addpath('URDFs')

% Load URDF
model = xml2struct(file);



end