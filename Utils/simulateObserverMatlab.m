%% Simulate with null points 
simple_test = false;
if simple_test
    N = 10;
    int = 0;
    q = zeros(nrOfJoints,N);
    qd = zeros(nrOfJoints,N);
    qdd = zeros(nrOfJoints,N);
    tau = zeros(nrOfJoints,N);
    r = zeros(nrOfJoints,N+1);
    r = mat2cell(r,[nrOfJoints], ones(1,N+1));
    for t =1:N
        res = integralOfpHatDotFunction('x0',0,...
                           'p',[q(:,t);qd(:,t);qdd(:,t);g;tau(:,t);r{:,t}]);
        int = int + res.xf;
        int_list(:,t)= full(int);
        r{:,t+1} = K*(pFunction(q(:,t),qd(:,t)) - int - pFunction(q(:,1),qd(:,1)));
    end
    rf = full([r{:}]);
else
    %% Load dataset of joint position, velocity and torques
    N = nrOfSamples;
    generalizedMomentum = zeros(N,nrOfJoints);
    r = zeros(nrOfJoints,N+1);
    r = mat2cell(r,[nrOfJoints], ones(1,N+1));
    int = 0;
    for t =1:N
        res = integralOfpHatDotFunction('x0',0,'p',[q(t,:)';qd(t,:)';qdd(t,:)';g;tau(t,:)';r{:,t}]);
        int = int + res.xf;
        r{:,t+1} = K*(pFunction(q(t,:),qd(t,:)) - int - pFunction(q(1,:),qd(1,:)));
        generalizedMomentum(t,:) = full(pFunction(q(t,:),qd(t,:)))';
    end
    rf = full([r{:}]);
    rf = rf(:,2:end)';
end
%% Plot result
plot_joint_trajectories(rf, timesInSeconds,'r(t)');