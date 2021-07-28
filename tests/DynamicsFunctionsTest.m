classdef DynamicsFunctionsTest < matlab.unittest.TestCase
    %DynamicsFunctionTest Test Robot Dynamics Model functions
    %   The functions used in computing the various components and quontities of the
    %   robot dynamics model againts the same or similar functionaties
    %   provided by iDynTree(see https://github.com/robotology/idyntree).
    
    properties
   
        location_tests_folder;
        location_generated_functions;
        testURDFfiles;
        
        % iDynTree structures and functions
        mdlLoader;
        kinDynComp;
        
        % Constants
        gravityModulus;
        nrOfTests;
        tol;
    end
    
    methods(TestMethodSetup)
        function setUpVariablesForTests(testCase)
            % Fix location of the folder to store the generated c and .mex files
            testCase.location_tests_folder = pwd;
            testCase.location_generated_functions = [testCase.location_tests_folder,'/../../automaticallyGeneratedFunctions'];

            % Fix location of urdf to be used in the tests
            twoLink_urdf = [testCase.location_tests_folder,'/../URDFs/twoLinks.urdf'];
            kuka_kr210 = [testCase.location_tests_folder,'/../URDFs/kuka_kr210.urdf'];
            %iCub_r_leg = [testCase.location_tests_folder,'/../URDFs/iCub_r_leg.urdf'];
            testCase.testURDFfiles = {twoLink_urdf,kuka_kr210};

            % Prepare iDynTree related variables and functions
            testCase.mdlLoader = iDynTree.ModelLoader();
            testCase.kinDynComp = iDynTree.KinDynComputations();

            % Gravity constant
            testCase.gravityModulus = 9.80665;

            % Number of random tests
            testCase.nrOfTests = 20;

            % Close to zero tolerance
            testCase.tol = 1e-10;
        end
    end
    
    methods(TestMethodTeardown)

    end
    
    methods(Test)
        function testInverseDynamics(testCase)
           
            % For each test URDF compute the Inverse Dynamics using urdf2casadi-matlab and iDynTree, then compare the results
            for r=1:length(testCase.testURDFfiles)
                
                robotModelURDF = testCase.testURDFfiles{r};
                
                % Load iDynTree model
                testCase.mdlLoader.loadModelFromFile(robotModelURDF);
                testCase.kinDynComp.loadRobotModel(testCase.mdlLoader.model());
                
                nrOfJoints = testCase.kinDynComp.model().getNrOfDOFs();
                
                % Compute the symbolic model
                symbolicDynamicFunction = urdf2casadi.Dynamics.symbolicInverseDynamics(robotModelURDF, false, testCase.location_generated_functions);
                
                % The external forces is a vector of (6,1). It has to be one per
                % link, expect the base link(which for now is considered fixed)
                extForce = zeros(6,nrOfJoints);
                g = [0, 0, -testCase.gravityModulus]';
                
                % Prepare variables to store results
                iDynResult_list = zeros(nrOfJoints, testCase.nrOfTests);
                symbolcResult_list = zeros(nrOfJoints, testCase.nrOfTests);
                
                % For each test compute a random pose
                for ii = 1 : testCase.nrOfTests
                    jointPos = rand(nrOfJoints,1);
                    jointVel = rand(nrOfJoints,1);
                    jointAcc = rand(nrOfJoints,1);

                    tau_symbolic_function = symbolicDynamicFunction(jointPos, jointVel, jointAcc, g, extForce);
                    tau_symbolic_function = full(tau_symbolic_function);

                    iDynResult_list(:,ii) = urdf2casadi.Utils.iDynTreeDynamicsFunctions.computeInverseDynamicsIDynTree(robotModelURDF,jointPos',jointVel',jointAcc', testCase.gravityModulus);
                    symbolcResult_list(:,ii)= tau_symbolic_function;
                end
            end

            assertLessThanOrEqual(testCase,abs(iDynResult_list'-symbolcResult_list'),  testCase.tol);
        end
        
        function testForwardDynamics(testCase)
            
            % For each test URDF compute the Inverse Dynamics using urdf2casadi-matlab and iDynTree, then compare the results
            for r=1:length(testCase.testURDFfiles)
                
                robotModelURDF = testCase.testURDFfiles{r};
                
                %Use Matlab Robotic Toolbox                                                                                                                               
                modelRobotMatlab = importrobot(robotModelURDF);                                                                                                                                                                                                                               
                modelRobotMatlab.DataFormat = 'column';                                                                              
                modelRobotMatlab.Gravity = [0 0 -testCase.gravityModulus]; 

                nrOfJoints = size(modelRobotMatlab.massMatrix,2);
                jointAccMatlab_list = zeros(nrOfJoints, testCase.nrOfTests);
                jointAccSymbolic_list = zeros(nrOfJoints, testCase.nrOfTests);

                for i = 1: testCase.nrOfTests
                    jointPos = rand(nrOfJoints,1);
                    jointVel = rand(nrOfJoints,1);
                    tau      = rand(nrOfJoints,1);

                    [jointAccMatlab, jointAccSymbolic] = computeIDyntreeFD_VS_Symblic(modelRobotMatlab, jointPos, jointVel, testCase.gravityModulus, tau, robotModelURDF);
                    jointAccMatlab_list(:,i) = jointAccMatlab;
                    jointAccSymbolic_list(:,i)= jointAccSymbolic;
                end

                assertLessThanOrEqual(testCase, abs(jointAccMatlab_list'-jointAccSymbolic_list'),  testCase.tol);
            end

        end
        
        function testInverseDynamicsInertiaParametersRegressor(testCase)
            
            % For each test URDF compute the Inverse Dynamics using urdf2casadi-matlab and iDynTree, then compare the results
            for r=1:length(testCase.testURDFfiles)
                robotModelURDF = testCase.testURDFfiles{r};
                
                % Rettrive Inertia parameters
                smds = urdf2casadi.Utils.modelExtractionFunctions.extractSystemModel(robotModelURDF);
                nrOfJoints = smds.NB;
                
                % Sumbolic Inverse Dyncamics function
                symbolicIDFunction = urdf2casadi.Dynamics.symbolicInverseDynamics(robotModelURDF, false, testCase.location_generated_functions);
                
                % Gravity column vector
                g =[0; 0; -testCase.gravityModulus];
                
                jointPos = rand(nrOfJoints,1);
                jointVel = rand(nrOfJoints,1);
                jointAcc = rand(nrOfJoints,1);
                
                % Test with symbolic function
                % If we do not supply the external forces they are NOT automatically considered null in the symbolic function
                nrOfJoints = size(jointPos,1);
                extForce = zeros(6,nrOfJoints);
                
                for ii = 1:nrOfJoints
                    p(:,ii) = [smds.mass{ii}; smds.mass{ii}*smds.com{ii}; smds.I{ii}(1,1); smds.I{ii}(1,2); smds.I{ii}(1,3);...
                                           smds.I{ii}(2,2); smds.I{ii}(2,3); smds.I{ii}(3,3)]; 
                end
                inertiaParameters = reshape(p,[],1);
                % Symbolic regressor
                Y = urdf2casadi.Utils.inverseDynamicsInertialParametersRegressor(robotModelURDF, false, testCase.location_generated_functions);

                tau_regressor_list = zeros(nrOfJoints, testCase.nrOfTests);
                tau_RNEA_list = zeros(nrOfJoints, testCase.nrOfTests);

                for ii = 1 : testCase.nrOfTests
                    tau_RNEA = symbolicIDFunction(jointPos, jointVel, jointAcc, g, extForce);
                    tau_RNEA = full(tau_RNEA);

                    tau_regressor = Y(jointPos, jointVel, jointAcc, g)*inertiaParameters;
                    tau_regressor = full(tau_regressor);

                    tau_regressor_list(:,ii) = tau_regressor;
                    tau_RNEA_list(:,ii) = tau_RNEA;
                end
                
                assertLessThanOrEqual(testCase, abs(tau_RNEA_list'-tau_regressor_list'),  testCase.tol);
            end
            
        end
    end
end

