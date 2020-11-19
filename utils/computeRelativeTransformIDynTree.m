
kuka_urdf = '/home/iiticublap041/idjl-model-identification/results/identification_results/kuka_kr30_ha/urdf/kr30_ha-identified.urdf';
location_tests_folder = pwd;
twoLink_urdf = [location_tests_folder,'/../URDFs/twoLinks.urdf'];
kuka_kr210 = [location_tests_folder,'/../URDFs/kuka_kr210.urdf'];
iCub_r_leg = [location_tests_folder,'/../URDFs/iCub_r_leg.urdf'];
irb5400 = '/home/iiticublap041/catkin_ws/src/abb/abb_irb5400_support/urdf/irb5400.urdf';
irb2400 = '/home/iiticublap041/catkin_ws/src/abb/abb_irb2400_support/urdf/irb2400.urdf';
simple_humanoid = '/home/iiticublap041/baljinder/urdf2casadi-matlab/URDFs/simple_humanoid.urdf';

%% input urdf file to acquire robot structure
robotModelURDF = kuka_kr210;

%% Extract robot from urdf 
smds = extractSystemModel(robotModelURDF);
% The model calibration helper is a loader for URDF files
mdlLoader = iDynTree.ModelCalibrationHelper();
mdlLoader.loadModelFromFile(robotModelURDF);

kinDynComp = iDynTree.KinDynComputations();
kinDynComp.loadRobotModel(mdlLoader.model());


for i = 1:6
    from_frame = kinDynComp.getFrameName(i-1);
    to_frame = kinDynComp.getFrameName(i);
    x = kinDynComp.getRelativeTransform(to_frame,from_frame);
    px = x.getPosition();
    rx = x.getRotation();
    posIDyn{i} = px.toMatlab;
    rotIDyn{i} = rx.toMatlab;
    
    x = x.asAdjointTransform();
    x = x.toMatlab();
    % Featherstone's notation considers first the angular part
    xx(4:6,1:3) = x(1:3,4:6);
    xx(1:3,4:6) = x(4:6,1:3);
    xx(1:3,1:3) = x(4:6,4:6);
    xx(4:6,4:6) = x(1:3,1:3);
    XupIDyn{i} = xx;

    xWorld = kinDynComp.getWorldTransform(i); 
    xWorld = xWorld.asAdjointTransform();
    xWorld = xWorld.toMatlab();
    xxW(4:6,1:3) = xWorld(1:3,4:6);
    xxW(1:3,4:6) = xWorld(4:6,1:3);
    xxW(1:3,1:3) = xWorld(4:6,4:6);
    xxW(4:6,4:6) = xWorld(1:3,1:3);
    XWorldIDyn{i} = xxW;
end
%%
mod = kinDynComp.model();
for j =1:7
    l = mod.getLink(j-1);
    I = l.getInertia();
    I = I.asMatrix;
    I = I.toMatlab;
    % Featherstone's notation considers first the angular part
    ii(4:6,1:3) = I(1:3,4:6);
    ii(1:3,4:6) = I(4:6,1:3);
    ii(1:3,1:3) = I(4:6,4:6);
    ii(4:6,4:6) = I(1:3,1:3);
    IiDyn{j} = ii;
end
