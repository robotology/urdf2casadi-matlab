function [outputTrajectories] = romualdi2020_generate_min_jerk_trajectories(initial_pos, initial_vel, initial_acc, final_pos, final_vel, final_acc, trajectoryDuration, samplingPeriod)
%romualdi2020_generate_min_jerk_trajectories Generate joint trajectories following the
%model of Romualdi 2020 of min jerk with arbitrary initial and final
%velocity and acceleration
vecSize = length(initial_pos);

assert(length(initial_vel) == vecSize);
assert(length(initial_acc) == vecSize);
assert(length(final_pos) == vecSize);
assert(length(final_vel) == vecSize);
assert(length(final_acc) == vecSize);

% For each vector component, we compute a quintic polynomial of the form
% p(t) = a0 + a1*t + a2*(t^2) + a3*(t^3) + a4*(t^4) + a5*(t^5) 

% Equations from 
% https://github.com/dic-iit/bipedal-locomotion-framework/blob/9c02814f2ae59a9773fd9bff6b34c1421d6381e6/src/Planners/src/QuinticSpline.cpp#L595
% theory from eq 5 of
% https://github.com/dic-iit/element_dcm-walking/issues/188#issuecomment-673542645 
x0 = initial_pos;
dx0 = initial_vel;
ddx0 = initial_acc;
xT = final_pos;
dxT = final_vel;
ddxT = final_acc;
T = trajectoryDuration;
a0 = x0;
a1 = dx0;
a2 = ddx0/2;
a3 = 1.0 / (T * T * T) ...
     * (x0 * 2.0E+1 - xT * 2.0E+1 + T * dx0 * 1.2E+1 + T * dxT * 8.0 + (T * T) * ddx0 * 3.0 ...
        - (T * T) * ddxT) ...
     * (-1.0 / 2.0);
a4 = (1.0 / (T * T * T * T) ...
     * (x0 * 3.0E+1 - xT * 3.0E+1 + T * dx0 * 1.6E+1 + T * dxT * 1.4E+1 ...
        + (T * T) * ddx0 * 3.0 - (T * T) * ddxT * 2.0)) ...
     / 2.0;
a5 = 1.0 / (T * T * T * T * T) ...
     * (x0 * 1.2E+1 - xT * 1.2E+1 + T * dx0 * 6.0 + T * dxT * 6.0 + (T * T) * ddx0 ...
        - (T * T) * ddxT) ...
     * (-1.0 / 2.0);

% Sample the trajectory 
t = 0:samplingPeriod:trajectoryDuration;
outputTrajectories = a0 + a1*t + a2*(t.^2) + a3*(t.^3) + a4*(t.^4) + a5*(t.^5);
outputTrajectories = outputTrajectories';
end

