
%% Fiala Tire Model Verification

clear;
clc;

% Test parameters
C_alpha = 5000;
F_z = 981;
mu = 0.6;

%% Test 1: Zero slip produces zero force

F_zero = fialaLateralForce(0, C_alpha, F_z, mu);

assert(abs(F_zero) < 1e-12, ...
    'Zero-slip test failed.');

%% Test 2: Force reverses when slip reverses

alpha = deg2rad(7);

F_positive = fialaLateralForce(alpha, C_alpha, F_z, mu);
F_negative = fialaLateralForce(-alpha, C_alpha, F_z, mu);

assert(abs(F_positive + F_negative) < 1e-10, ...
    'Force symmetry test failed.');

%% Test 3: Small-slip behavior approaches linear stiffness

alpha_small = 1e-8;

F_small = fialaLateralForce( ...
    alpha_small, C_alpha, F_z, mu);

relative_error = abs( ...
    F_small/(C_alpha*alpha_small) - 1);

assert(relative_error < 1e-3, ...
    'Small-slip linearity test failed.');

%% Test 4: Force never exceeds friction limit

alpha_values = linspace(-deg2rad(45), deg2rad(45), 1001);

force_values = arrayfun( ...
    @(a) fialaLateralForce(a, C_alpha, F_z, mu), ...
    alpha_values);

assert(all(abs(force_values) <= mu*F_z + 1e-9), ...
    'Friction limit test failed.');

%% Test 5: Force is continuous at saturation boundary

eps_s = 1e-5;

alpha_below = atan((3-eps_s)*mu*F_z/C_alpha);
alpha_above = atan((3+eps_s)*mu*F_z/C_alpha);

F_below = fialaLateralForce( ...
    alpha_below, C_alpha, F_z, mu);

F_above = fialaLateralForce( ...
    alpha_above, C_alpha, F_z, mu);

assert(abs(F_below-F_above) < 1e-5, ...
    'Saturation continuity test failed.');

fprintf('\nAll five Fiala tire model tests passed.\n');
