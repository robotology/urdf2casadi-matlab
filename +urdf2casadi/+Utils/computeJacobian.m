function J = computeJacobian(k_X_N, Xup,S)
X = k_X_N;
N = max(size(S));
for i= N:-1:1
    J{i} = X*S{i};
    if i>1
        X = X*Xup{i};
    end
end
end