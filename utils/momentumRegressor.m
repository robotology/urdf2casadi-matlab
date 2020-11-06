function reg = momentumRegressor(v)
reg = [v(4:6),    skew(v(1:3)),  zeros(3,6);
       zeros(3,1),-skew(v(1:3)), computeRotationalMomentumRegressor(v(1:3))];
end
