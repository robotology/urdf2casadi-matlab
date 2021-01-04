%Compute the reduced model urdf model(without sensors)
location_current_folder = pwd;
iCub09 = [location_current_folder,'/../../../URDFs/iCubGenova9.urdf'];
%% input urdf file to acquire robot structure
robotModelURDF = iCub09;
% The model calibration helper is a loader for URDF files
mdlLoader = iDynTree.ModelLoader();
% Parsing options
parseOpts = iDynTree.ModelParserOptions;
parseOpts.addSensorFramesAsAdditionalFrames(false);
mdlLoader.setParsingOptions(parseOpts);
% load the list of joints to be used in the reduced model
jointList_idyntree     = iDynTree.StringVector();
jointsToLoad = {'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};

for k = 1:length(jointsToLoad)
    jointList_idyntree.push_back(jointsToLoad{k});
end

mdlLoader.loadReducedModelFromFile(robotModelURDF,jointList_idyntree,'urdf');
% The reduced model
reducedModel = mdlLoader.model();
sensors = mdlLoader.sensors();
% options for model expoter
opts = iDynTree.ModelExporterOptions();
opts.baseLink('root_link');
opts.exportFirstBaseLinkAdditionalFrameAsFakeURDFBase(false);
modelExporter = iDynTree.ModelExporter();
modelExporter.init(reducedModel,sensors,opts);
locationReducedModel = '../../../URDFs/iCub_r_leg.urdf';
modelExporter.exportModelToFile(locationReducedModel)
