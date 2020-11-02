function L = computeRotationalMomentumRegressor(w)
%Compute the 3x6 rectangular matix regressor for the rotational inertia
L = [w(1), w(2), w(3),  0,    0,   0;
      0,   w(1),  0,   w(2), w(3), 0;
      0,    0,   w(1),  0,   w(2), w(3)];
end