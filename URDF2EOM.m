function [model] = URDF2EOM(file)
addpath('xml2struct')
addpath('URDFs')

% Load URDF
model = xml2struct(file);



end