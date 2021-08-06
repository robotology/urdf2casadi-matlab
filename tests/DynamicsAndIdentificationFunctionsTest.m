classdef DynamicsAndIdentificationFunctionsTest < matlab.unittest.TestCase
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
            testCase.location_generated_functions = [testCase.location_tests_folder,'/../automaticallyGeneratedFunctions'];

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
            % Test the Inverse dynamics
            
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
            % Test the forward dynamics
            
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
            % Test the inverse dynamics Inertia Parameter regressor
            
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
                
                % Test with symbolic function
                % If we do not supply the external forces they are NOT automatically considered null in the symbolic function
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
                    
                    jointPos = rand(nrOfJoints,1);
                    jointVel = rand(nrOfJoints,1);
                    jointAcc = rand(nrOfJoints,1);
                
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
        
        function testInverseDynamicsInertiaParametersRegressor_2(testCase)
            % A second test on the Inverse Dynamics regressor
            
            % Import necessary functions
            import urdf2casadi.Utils.inverseDynamicsInertialParametersRegressor
            import urdf2casadi.Utils.iDynTreeDynamicsFunctions.computeRegressorIdynTree
                
            for r = 1 : length(testCase.testURDFfiles)
                
                robotURDFModel = testCase.testURDFfiles{r};

                % Load iDynTree model
                testCase.mdlLoader.loadModelFromFile(robotURDFModel);
                testCase.kinDynComp.loadRobotModel(testCase.mdlLoader.model());

                nrOfJoints = testCase.kinDynComp.model().getNrOfDOFs();
            
                Y = inverseDynamicsInertialParametersRegressor(robotURDFModel, true, testCase.location_generated_functions);
                
                g = [0; 0; -testCase.gravityModulus];
                
                for t = 1: testCase.nrOfTests
                    jointPos = rand(nrOfJoints,1);
                    jointVel = rand(nrOfJoints,1);
                    jointAcc = rand(nrOfJoints,1);
                    
                    Y_symb = Y(jointPos, jointVel, jointAcc, g);

                    Y_IDyn = computeRegressorIdynTree(robotURDFModel, jointPos, jointVel, jointAcc, testCase.gravityModulus);

                    regressor_matrix_error = abs(Y_IDyn(:,11:end)-full(Y_symb));
                    
                    assertLessThanOrEqual(testCase, regressor_matrix_error,  testCase.tol);
                end
            end
        end
        function testMassAndCoriolisMatrices(testCase)
            % Test the Mass and the Coriolis matrices

            %% Import necesary functions
            import urdf2casadi.Utils.modelExtractionFunctions.extractSystemModel
            import urdf2casadi.Dynamics.symbolicInverseDynamics
            import urdf2casadi.Utils.iDynTreeDynamicsFunctions.computeMassMatrixIDynTree
            import urdf2casadi.Utils.iDynTreeDynamicsFunctions.computeGravityTorqueIDynTree
            import urdf2casadi.Utils.iDynTreeDynamicsFunctions.computeGeneralizedBiasForceIDynTree
            import urdf2casadi.Dynamics.HandC
            import urdf2casadi.Dynamics.auxiliarySymbolicDynamicsFunctions.computeSymbolicCoriolismatrix
            import urdf2casadi.Dynamics.auxiliarySymbolicDynamicsFunctions.computeGravityTorque
            import urdf2casadi.Utils.auxiliaryFunctions.cell2mat_casadi
            
            for r = 1 : length(testCase.testURDFfiles)
               
                robotURDFModel = testCase.testURDFfiles{r};
                
                % Inverse dynamics symbolic function
                symbolicIDFunction = symbolicInverseDynamics(robotURDFModel, false, testCase.location_generated_functions);

                % Compute mass matrix, its derivative and Coriolis matrix with an efficient algorithm
                smds = extractSystemModel(robotURDFModel);
                nrOfJoints = smds.NB;

                g = [0;0;-testCase.gravityModulus];

                % Prepare variables to store results
                e_massMatrix = zeros(nrOfJoints,nrOfJoints, testCase.nrOfTests);
                e_gravity    = zeros(nrOfJoints, testCase.nrOfTests);
                e_torqueID   = zeros(nrOfJoints, testCase.nrOfTests);
                e_genBias    = zeros(nrOfJoints, testCase.nrOfTests);
                
                for ii = 1 : testCase.nrOfTests
                    %% Random points
                    jointPos = rand(nrOfJoints,1);
                    jointVel = rand(nrOfJoints,1);
                    jointAcc = rand(nrOfJoints,1);

                    %% IDynTree values 
                    % Compute mass matrix with IDynTree
                    M_IDyn = computeMassMatrixIDynTree(robotURDFModel, jointPos, jointVel, jointAcc, testCase.gravityModulus);
                    % Compute gravity torques IDynTree
                    tau_gravity_IDyn = computeGravityTorqueIDynTree(robotURDFModel, jointPos, testCase.gravityModulus);
                    % Compute the generalized bias force C(q,qd)*qd + g(q)
                    genealizedBias_IDyn = computeGeneralizedBiasForceIDynTree(robotURDFModel, jointPos, jointVel, testCase.gravityModulus);

                    %% Symbolic Matrices
                    [H_cell,HDot_cell,C_cell] = computeSymbolicCoriolismatrix(jointPos, jointVel, jointAcc, smds);
                    H = cell2mat_casadi(H_cell);
                    C = cell2mat_casadi(C_cell);

                    % Use CRBA and NE
                    [H_CRBA,C_CRBA] = HandC( smds, jointPos,jointVel, g );

                    %% Compute gravity torques using the symbolic Inverse Dynamics function
                    tau_gravity = full(computeGravityTorque(jointPos, g, symbolicIDFunction));

                    %% Compare generalized bias forces
                    generalizedBias = C*jointVel + tau_gravity;

                    %% Compute the inverse dynamics with the matrices  
                    tau_efficient = H*jointAcc + C*jointVel + tau_gravity;
                    tau_ID = full(symbolicIDFunction(jointPos,jointVel,jointAcc,g,0));

                    %% Compare iDynTee and symbolic 
                    e_massMatrix(:,:,ii) = abs(H - M_IDyn);
                    e_gravity(:,ii) = abs(tau_gravity - tau_gravity_IDyn);
                    e_torqueID(:,ii) = abs(tau_efficient-tau_ID);
                    e_genBias(:,ii) = abs(generalizedBias-genealizedBias_IDyn);
                end
                
                assertLessThanOrEqual(testCase, e_massMatrix,  testCase.tol);
                assertLessThanOrEqual(testCase, e_gravity,  testCase.tol);
                assertLessThanOrEqual(testCase, e_torqueID,  testCase.tol);
                assertLessThanOrEqual(testCase, e_genBias,  testCase.tol);
            
            end
        end
        
        function testSingleLinkRegressor(testCase)
            %% Import necesary functions
            import urdf2casadi.Utils.modelExtractionFunctions.extractSystemModel
            import urdf2casadi.Utils.computeKinematics
            import urdf2casadi.Identification.auxiliarySymbolicDynamicsFunctions.computeLinkRegressor
            import urdf2casadi.Utils.Spatial.crf

            for r = 1 : length(testCase.testURDFfiles)
                
                robotURDFModel = testCase.testURDFfiles{r};
                
                %Load urdf and convert to SMDS format
                smds = extractSystemModel(robotURDFModel);
                nrOfJoints = smds.NB;

                g = [0; 0; -testCase.gravityModulus];

                for t = 1 : testCase.nrOfTests
                    jointPos = rand(nrOfJoints, 1);
                    jointVel = rand(nrOfJoints, 1);
                    jointAcc = rand(nrOfJoints, 1);

                    % Compute frame transformations, joint motion subspace matrix, link velocity and accelerations
                    [X,~,~,Xup, v, a] = computeKinematics(smds, jointPos, jointVel, jointAcc, g);

                    % Compute for each link i the matrix A_j for all links j distal to link
                    % i. This matix arises from recasting the spatial Newton-Euler dynamic equation 
                    % of each link i in a form linear wrt the Inertial paramter vector of link i, expressed in the local frame i 
                    for l = 1:smds.NB
                        % Gravity in local frame
                        g_l = (X{1}{1,l}*Xup{1})*[0;0;0;g];
                        % Velocity and acceleration must be expressed in the same frame as the
                        % inertia and the center of mass: the body i local frame
                        A{l} = computeLinkRegressor( v{l}, a{l});
                        % Test the regressor 
                        p = [smds.mass{l}; smds.mass{l}*smds.com{l}; smds.I{l}(1,1); smds.I{l}(1,2); smds.I{l}(1,3);...
                                               smds.I{l}(2,2); smds.I{l}(2,3); smds.I{l}(3,3)];
                        f_regressor = A{l}*p;
                        f_NE = smds.I{l}*a{l} + crf(v{l})*smds.I{l}*v{l};   
                    end

                    % Check if the Newton-Euler and regressor based dynamics equations give the
                    % same result
                    assertLessThanOrEqual(testCase, f_regressor - f_NE,  testCase.tol);
                end
            end
        end
        
        function testJacobian(testCase)

            for r = 1 : length(testCase.testURDFfiles)
                
                robotURDFModel = testCase.testURDFfiles{r};
                
                opts.geneate_c_code = true;
                opts.location_generated_fucntion = testCase.location_generated_functions;
                opts.FrameVelocityRepresentation = "MIXED_REPRESENTATION";
                [J_symb,X,XForce,S,O_X_ee] = urdf2casadi.Dynamics.auxiliarySymbolicDynamicsFunctions.createSpatialTransformsFunction(robotURDFModel, opts);

                % IDynTree variables
                testCase.mdlLoader.loadModelFromFile(robotURDFModel);
                testCase.kinDynComp.loadRobotModel(testCase.mdlLoader.model());

                % Set the body-fixed frame (left-trivialized velocity) representation
                testCase.kinDynComp.setFrameVelocityRepresentation(iDynTree.MIXED_REPRESENTATION);
                model_iDyn = testCase.kinDynComp.model();

                nrOfJoints = model_iDyn.getNrOfJoints();
                q_idyn = iDynTree.VectorDynSize(nrOfJoints);
                dq_idyn = iDynTree.VectorDynSize(nrOfJoints);
                grav = iDynTree.Vector3();
                J_idyn = iDynTree.MatrixDynSize();


                % Constants
                % Assume that the base frame has index 0 and the
                % end-effector frame has index nrOfJoints
                baseFrame = 0;
                eeFrame = nrOfJoints;
                grav.fromMatlab([0; 0; -testCase.gravityModulus]);
                
                J_iDyn_list = zeros(6,nrOfJoints, testCase.nrOfTests);
                J_symb_list = zeros(6,nrOfJoints, testCase.nrOfTests);
                e_jac = zeros(1, testCase.nrOfTests);
                
                for t = 1 : testCase.nrOfTests

                    jointPos = rand(nrOfJoints,1);

                    % Compute jacobian with iDynTree
                    q_idyn.fromMatlab(jointPos);
                    dq_idyn.fromMatlab(zeros(nrOfJoints,1));
                    

                    testCase.kinDynComp.setRobotState(q_idyn, dq_idyn, grav);
                    testCase.kinDynComp.getRelativeJacobian(baseFrame, eeFrame, J_idyn);
                    J_tmp = J_idyn.toMatlab();

                    % Featherstone's notation considers first the angular part
                    J_iDyn_list(4:6,:,t) = J_tmp(1:3,:);
                    J_iDyn_list(1:3,:,t) = J_tmp(4:6,:);

                    % Compute Jacobian with symbolic function
                    J_symb_list(:,:,t) = full(J_symb(jointPos));

                    % Error in the norm of the difference of the Jacobians
                    e_jac(:,t)  = norm(J_symb_list(:,:,t) - J_iDyn_list(:,:,t));

                end
                
                assertLessThanOrEqual(testCase, e_jac,  testCase.tol);
            end
        end
    end
end

