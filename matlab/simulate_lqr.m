%% LQR Controller Design
function results_nonlinear = simulate_lqr( ...
    vx, initial_offset, Q, R, vx_design, ...
    make_plots, mu, dt, controller_dt, ...
    controller_type, lookahead)

if nargin < 1 || isempty(vx), vx = 5.0; end
if nargin < 2 || isempty(initial_offset), initial_offset = 0.25; end
if nargin < 3 || isempty(Q), Q = diag([1, 1, 70, 15]); end
if nargin < 4 || isempty(R), R = 3; end
if nargin < 5 || isempty(vx_design), vx_design = 5.0; end
if nargin < 6 || isempty(make_plots), make_plots = false; end
if nargin < 7 || isempty(mu), mu = 1.0; end
if nargin < 8 || isempty(dt), dt = 0.01; end
if nargin < 9 || isempty(controller_dt)
    controller_dt = dt;
end

if nargin < 10 || isempty(controller_type)
    controller_type = 'lqr';
end

if nargin < 11 || isempty(lookahead)
    lookahead = 3.0;
end

% Validate controller selection
if ~ischar(controller_type)
    error('controller_type must be ''lqr'' or ''pure_pursuit''.');
end

controller_type = lower(strtrim(controller_type));

if ~strcmp(controller_type, 'lqr') && ...
        ~strcmp(controller_type, 'pure_pursuit')
    error('Unsupported controller type.');
end

% Validate Pure Pursuit lookahead distance
if ~isscalar(lookahead) || ~isfinite(lookahead) || lookahead <= 0
    error('Lookahead must be a positive finite scalar.');
end

% Vehicle parameters
m  = 200;      % Mass (kg)
Iz = 250;      % Yaw moment of inertia (kg*m^2)
lf = 1.25;     % CG to front axle (m)
lr = 1.25;     % CG to rear axle (m)
Cf = 5000;     % Front cornering stiffness (N/rad)
Cr = 5000;     % Rear cornering stiffness (N/rad)

% Tire and road parameters
g = 9.81;     % Gravitational acceleration (m/s^2)

% Static axle normal loads
F_zf = m * g * lr / (lf + lr);
F_zr = m * g * lf / (lf + lr);

% Validate simulation inputs
if ~isscalar(vx) || ~isfinite(vx) || vx <= 0
    error('Vehicle speed vx must be a positive finite scalar.');
end

if ~isscalar(vx_design) || ~isfinite(vx_design) || vx_design <= 0
    error('Design speed vx_design must be a positive finite scalar.');
end

if ~isscalar(mu) || ~isfinite(mu) || mu <= 0
    error('Friction coefficient mu must be a positive finite scalar.');
end

if ~isscalar(dt) || ~isfinite(dt) || dt <= 0
    error('Timestep dt must be a positive finite scalar.');
end

if ~isscalar(controller_dt) || ...
        ~isfinite(controller_dt) || controller_dt <= 0
    error('Controller timestep must be a positive finite scalar.');
end

% Number of plant integration steps per controller update
steps_per_control = round(controller_dt / dt);

if steps_per_control < 1 || ...
        abs(steps_per_control * dt - controller_dt) > 1e-10
    error(['Controller timestep must be an integer multiple ', ...
        'of the plant integration timestep.']);
end

% State-space model
% States: [vy; yaw_rate; lateral_error; heading_error]
% Input: front-wheel steering angle (rad)

A = [-(Cf+Cr)/(m*vx), ...
    -((Cf*lf-Cr*lr)/(m*vx) + vx), 0, 0;
    -(Cf*lf-Cr*lr)/(Iz*vx), ...
    -(Cf*lf^2+Cr*lr^2)/(Iz*vx), 0, 0;
    1, 0, 0, vx;
    0, 1, 0, 0];

B = [Cf/m;
    Cf*lf/Iz;
    0;
    0];

% Nominal model used to design the controller at vx_design
A_design = [-(Cf+Cr)/(m*vx_design), ...
    -((Cf*lf-Cr*lr)/(m*vx_design) + vx_design), 0, 0;
    -(Cf*lf-Cr*lr)/(Iz*vx_design), ...
    -(Cf*lf^2+Cr*lr^2)/(Iz*vx_design), 0, 0;
    1, 0, 0, vx_design;
    0, 1, 0, 0];

B_design = [Cf/m;
    Cf*lf/Iz;
    0;
    0];

% Linearized dynamic bicycle model around straight-line driving.
% States: [vy; yaw_rate; lateral_error; heading_error]
% Curved-path tracking will introduce a curvature-dependent reference.

if strcmp(controller_type, 'lqr')

    % Calculate optimal feedback gains
    K = lqr(A_design, B_design, Q, R);

    % Evaluate the linearized closed-loop model
    closed_loop_eigenvalues = eig(A - B*K);

    disp('Linearized closed-loop eigenvalues:');
    disp(closed_loop_eigenvalues);

    disp('State-space matrix A:');
    disp(A);

    disp('Input matrix B:');
    disp(B);

    disp('LQR feedback gains K:');
    disp(K);

else

    % Pure Pursuit does not use LQR feedback gains
    K = [];
    closed_loop_eigenvalues = [];

end

% Simulation parameters
T_requested = 50 / vx;
t = 0:dt:T_requested;
T = t(end);

tracking_start_time = 0.2 * T;

% Reference path
% Use fixed spatial resolution independent of plant timestep.
path_ds = 0.01;  % Reference point spacing (m)

% Extend path beyond the simulation endpoint so Pure Pursuit
% can maintain its lookahead distance throughout the simulation.
path_end = vx*T + lookahead + 1.0;

x_ref = 0:path_ds:path_end;
y_ref = 2*sin(x_ref / 5);

% Reference path heading
dy_dx = (2/5) * cos(x_ref / 5);
psi_ref = atan(dy_dx);

% Reference path curvature
d2y_dx2 = -(2/25) * sin(x_ref / 5);
kappa = d2y_dx2 ./ (1 + dy_dx.^2).^(3/2);

% Actual vehicle lateral velocity and yaw rate
vy_history = zeros(1, length(t));
r_history = zeros(1, length(t));

% LQR tracking-error state:
% [vy_error; yaw_rate_error; lateral_error; heading_error]
x = zeros(4, length(t));

% Initial actual vehicle motion
vy_history(1) = 0;
r_history(1) = 0;

% Initial reference curvature is zero at X = 0
r_ref_initial = 0;

reference_solution_initial = ...
    [A(1,1), B(1);
    A(2,1), B(2)] \ ...
    (-[A(1,2); A(2,2)] * r_ref_initial);

vy_ref_initial = reference_solution_initial(1);

% Initial LQR tracking-error state
x(1,1) = vy_history(1) - vy_ref_initial;
x(2,1) = r_history(1) - r_ref_initial;

% Store steering commands
delta_history = zeros(1, length(t));

% Global vehicle state history
X_pos = zeros(1, length(t));
Y_pos = zeros(1, length(t));
psi = zeros(1, length(t));

% Store path-tracking errors
ey_history = zeros(1, length(t));
epsi_history = zeros(1, length(t));

% Initial global pose
X_pos(1) = 0;
Y_pos(1) = y_ref(1) + initial_offset;
psi(1) = psi_ref(1);

% Steering actuator constraints
delta_max = 0.5;          % Maximum steering angle (rad)
delta_rate_max = 2.0;     % Maximum steering rate (rad/s)

delta_command_history = zeros(1, length(t));
delta_prev = 0;            % Initial applied steering angle (rad)

delta_rate_limited_history = false(1, length(t));

% Find the closest point on the continuous reference path
[x_closest_initial, y_closest_initial] = ...
    closestPathPoint(X_pos(1), Y_pos(1), x_ref, y_ref);

% Reference heading at the closest point
dy_dx_initial = (2/5) * cos(x_closest_initial / 5);
psi_ref_initial = atan(dy_dx_initial);

% Displacement from the closest reference point
dx_initial = X_pos(1) - x_closest_initial;
dy_initial = Y_pos(1) - y_closest_initial;

% Signed perpendicular tracking error
ey_history(1) = -dx_initial*sin(psi_ref_initial) ...
    + dy_initial*cos(psi_ref_initial);

% Wrapped heading error
epsi_history(1) = atan2( ...
    sin(psi(1) - psi_ref_initial), ...
    cos(psi(1) - psi_ref_initial));

% Initialize tracking-error states consistently
x(3,1) = ey_history(1);
x(4,1) = epsi_history(1);

delta_ref_history = zeros(1, length(t));

% Store front and rear tire slip angles
alpha_f_history = zeros(1, length(t)-1);
alpha_r_history = zeros(1, length(t)-1);

% Store front and rear lateral tire forces
F_yf_history = zeros(1, length(t)-1);
F_yr_history = zeros(1, length(t)-1);

% Hold controller commands between controller updates
delta_cmd_hold = 0;
delta_ref_hold = 0;

for i = 1:length(t)-1


    if mod(i-1, steps_per_control) == 0

        % Find the closest point on the continuous reference path.
        [x_closest_current, ~] = closestPathPoint( ...
            X_pos(i), Y_pos(i), x_ref, y_ref);

        if strcmp(controller_type, 'lqr')

            % Reference heading and curvature at the closest point
            dy_dx_current = (2/5) * cos(x_closest_current / 5);
            d2y_dx2_current = -(2/25) * sin(x_closest_current / 5);

            psi_ref_current = atan(dy_dx_current);

            kappa_current = d2y_dx2_current / ...
                (1 + dy_dx_current^2)^(3/2);

            % Desired yaw rate for the local path curvature
            r_ref = vx * kappa_current;

            % Solve for reference lateral velocity and steering angle
            reference_solution = ...
                [A(1,1), B(1);
                A(2,1), B(2)] \ ...
                (-[A(1,2); A(2,2)] * r_ref);

            vy_ref = reference_solution(1);
            delta_ref = reference_solution(2);

            % Signed perpendicular tracking error
            dx_current = X_pos(i) - x_closest_current;
            dy_current = Y_pos(i) - ...
                2*sin(x_closest_current / 5);

            ey = -dx_current*sin(psi_ref_current) ...
                + dy_current*cos(psi_ref_current);

            % Account for reference sideslip
            beta_ref = atan2(vy_ref, vx);
            psi_body_ref = psi_ref_current - beta_ref;

            % Wrapped heading error
            epsi = atan2(sin(psi(i) - psi_body_ref), ...
                cos(psi(i) - psi_body_ref));

            % LQR feedback state
            x_feedback = [vy_history(i) - vy_ref;
                r_history(i) - r_ref;
                ey;
                epsi];

            % Feedforward + LQR feedback
            delta_cmd_hold = delta_ref - K*x_feedback;
            delta_ref_hold = delta_ref;

        else

            % Calculate rear axle position from the CG position.
            X_rear = X_pos(i) - lr*cos(psi(i));
            Y_rear = Y_pos(i) - lr*sin(psi(i));

            % Find the closest reference point to the rear axle.
            [x_closest_rear, ~] = closestPathPoint( ...
                X_rear, Y_rear, x_ref, y_ref);

            % Pure Pursuit steering command using rear axle geometry.
            delta_cmd_hold = pure_pursuit_controller( ...
                X_rear, Y_rear, psi(i), ...
                x_closest_rear, x_ref, y_ref, ...
                lf + lr, lookahead);

            % Pure Pursuit does not use LQR feedforward steering.
            delta_ref_hold = 0;

        end
    end


    % Hold the most recent controller command
    delta_command_history(i) = delta_cmd_hold;
    delta_ref_history(i) = delta_ref_hold;

    % Enforce steering angle limit
    delta_angle_limited = min(max(delta_cmd_hold, -delta_max), delta_max);

    % Enforce steering rate limit
    max_delta_change = delta_rate_max * dt;

    delta = delta_prev + min(max( ...
        delta_angle_limited - delta_prev, ...
        -max_delta_change), max_delta_change);

    % Check whether the requested angle exceeds the rate limit
    delta_rate_limited_history(i) = ...
        abs(delta_angle_limited - delta_prev) > max_delta_change;

    % Store applied steering angle
    delta_history(i) = delta;

    % Update previous applied steering for the next time step
    delta_prev = delta;

    % Nonlinear dynamic bicycle model

    % Current vehicle states
    vy = vy_history(i);
    r = r_history(i);

    % Front and rear tire slip angles
    alpha_f = delta - atan2(vy + lf*r, vx);
    alpha_r = -atan2(vy - lr*r, vx);

    alpha_f_history(i) = alpha_f;
    alpha_r_history(i) = alpha_r;

    % Nonlinear Fiala lateral tire-force model
    F_yf = fialaLateralForce(alpha_f, Cf, F_zf, mu);
    F_yr = fialaLateralForce(alpha_r, Cr, F_zr, mu);

    % Store tire forces
    F_yf_history(i) = F_yf;
    F_yr_history(i) = F_yr;

    % Project front tire lateral force into vehicle coordinates
    F_yf_vehicle = F_yf * cos(delta);

    % Lateral acceleration
    vy_dot = (F_yf_vehicle + F_yr)/m - vx*r;

    % Yaw angular acceleration
    r_dot = (lf*F_yf_vehicle - lr*F_yr)/Iz;

    % Collect state derivatives
    vehicle_derivative = [vy_dot; r_dot];

    % Update actual vehicle motion
    vy_history(i+1) = vy_history(i) ...
        + vehicle_derivative(1)*dt;

    r_history(i+1) = r_history(i) ...
        + vehicle_derivative(2)*dt;

    % Update global position using actual vehicle motion
    X_dot = vx*cos(psi(i)) - vy_history(i)*sin(psi(i));
    Y_dot = vx*sin(psi(i)) + vy_history(i)*cos(psi(i));

    X_pos(i+1) = X_pos(i) + X_dot*dt;
    Y_pos(i+1) = Y_pos(i) + Y_dot*dt;
    psi(i+1) = psi(i) + r_history(i)*dt;

    % Find the closest point on the continuous reference path
    [x_closest_next, y_closest_next] = ...
        closestPathPoint(X_pos(i+1), Y_pos(i+1), x_ref, y_ref);

    % Reference heading at the closest point
    dy_dx_next = (2/5) * cos(x_closest_next / 5);
    psi_ref_next = atan(dy_dx_next);

    % Displacement from the closest reference point
    dx = X_pos(i+1) - x_closest_next;
    dy = Y_pos(i+1) - y_closest_next;

    % Signed perpendicular tracking error
    ey_history(i+1) = -dx*sin(psi_ref_next) ...
        + dy*cos(psi_ref_next);

    % Calculate reference motion at the updated position
    dy_dx_next = (2/5) * cos(x_closest_next / 5);
    d2y_dx2_next = -(2/25) * sin(x_closest_next / 5);

    kappa_next = d2y_dx2_next / ...
        (1 + dy_dx_next^2)^(3/2);

    r_ref_next = vx * kappa_next;

    reference_solution_next = ...
        [A(1,1), B(1);
        A(2,1), B(2)] \ ...
        (-[A(1,2); A(2,2)] * r_ref_next);

    vy_ref_next = reference_solution_next(1);

    % Reference sideslip at the updated position
    beta_ref_next = atan2(vy_ref_next, vx);

    % Desired body heading at the updated position
    psi_body_ref_next = psi_ref_next - beta_ref_next;

    % Wrapped body-heading error
    epsi_history(i+1) = atan2( ...
        sin(psi(i+1) - psi_body_ref_next), ...
        cos(psi(i+1) - psi_body_ref_next));

    % Store all four tracking-error states
    x(1,i+1) = vy_history(i+1) - vy_ref_next;
    x(2,i+1) = r_history(i+1) - r_ref_next;
    x(3,i+1) = ey_history(i+1);
    x(4,i+1) = epsi_history(i+1);

end

% Find the timestep with maximum front lateral force
[~, front_force_idx] = max(abs(F_yf_history));

% Calculate normalized slip at that timestep
alpha_front_peak = alpha_f_history(front_force_idx);

s_front_peak = Cf * tan(alpha_front_peak) ...
    / (mu * F_zf);

% Calculate relative incremental stiffness
if abs(s_front_peak) >= 3

    % Tire has reached the saturation plateau
    front_stiffness_ratio_tan = 0;
    front_stiffness_ratio_alpha = 0;

else

    % Relative slope with respect to tan(alpha)
    front_stiffness_ratio_tan = ...
        (1 - abs(s_front_peak)/3)^2;

    % Relative slope with respect to alpha
    front_stiffness_ratio_alpha = ...
        front_stiffness_ratio_tan / cos(alpha_front_peak)^2;

end

fprintf('\nFront Tire Nonlinearity Diagnostics\n');

fprintf('Slip angle at peak force: %.3f deg\n', ...
    rad2deg(alpha_front_peak));

fprintf('Normalized slip parameter: %.4f\n', ...
    abs(s_front_peak));

fprintf('Force utilization: %.2f%%\n', ...
    100 * abs(F_yf_history(front_force_idx))/(mu*F_zf));

fprintf('Relative dFy/dtan(alpha): %.2f%%\n', ...
    100 * front_stiffness_ratio_tan);

fprintf('Relative dFy/dalpha: %.2f%%\n', ...
    100 * front_stiffness_ratio_alpha);

if make_plots
    figure;
    
    plot(t(1:end-1), rad2deg(alpha_f_history), ...
        'LineWidth', 1.5);
    hold on;
    
    plot(t(1:end-1), rad2deg(alpha_r_history), ...
        'LineWidth', 1.5);
    
    xlabel('Time (s)');
    ylabel('Tire slip angle (deg)');
    title('Front and Rear Tire Slip Angles');
    legend('Front', 'Rear');
    grid on;
    hold off;
end

[max_front_slip, front_idx] = max(abs(alpha_f_history));
[max_rear_slip, rear_idx] = max(abs(alpha_r_history));

fprintf('Maximum front slip occurs at %.3f s\n', t(front_idx));
fprintf('Maximum rear slip occurs at %.3f s\n', t(rear_idx));

max_front_slip = max(abs(alpha_f_history));
max_rear_slip = max(abs(alpha_r_history));

fprintf('\nTire Slip Angle Diagnostics\n');

fprintf('Maximum front slip angle: %.4f deg\n', ...
    rad2deg(max_front_slip));

fprintf('Maximum rear slip angle: %.4f deg\n', ...
    rad2deg(max_rear_slip));

fprintf('Time rate-limited: %.2f %%\n', ...
    100 * mean(delta_rate_limited_history(1:end-1)));

startup_idx = t < tracking_start_time;

startup_max_ey = max(abs(ey_history(startup_idx)));

fprintf('Maximum startup cross-track error: %.4f m\n', ...
    startup_max_ey);

% Settling time: error remains below threshold for the rest of the run
settling_threshold = 0.1;  % meters

% Find the final sample outside the settling threshold
last_outside_idx = find( ...
    abs(ey_history) >= settling_threshold, 1, 'last');

if isempty(last_outside_idx)

    settling_idx = 1;

elseif last_outside_idx == length(t)

    settling_idx = [];

else

    settling_idx = last_outside_idx + 1;

end

if isempty(settling_idx)
    fprintf('Cross-track settling time: Not settled within simulation\n');
else
    fprintf('Cross-track settling time: %.2f s\n', t(settling_idx));
end

% Path-tracking performance metrics
rms_ey = sqrt(mean(ey_history.^2));
max_abs_ey = max(abs(ey_history));

rms_epsi = sqrt(mean(epsi_history.^2));
max_abs_epsi = max(abs(epsi_history));

% Steering effort
rms_delta = sqrt(mean(delta_history(1:end-1).^2));
max_abs_delta = max(abs(delta_history(1:end-1)));

fprintf('\n%s Performance Metrics\n', upper(controller_type));
fprintf('RMS cross-track error: %.4f m\n', rms_ey);
fprintf('Maximum cross-track error: %.4f m\n', max_abs_ey);
fprintf('RMS heading error: %.4f deg\n', rad2deg(rms_epsi));
fprintf('Maximum heading error: %.4f deg\n', rad2deg(max_abs_epsi));
fprintf('RMS steering angle: %.4f deg\n', rad2deg(rms_delta));
fprintf('Maximum steering angle: %.4f deg\n', rad2deg(max_abs_delta));

% Steering constraint diagnostics
% Steering constraint diagnostics for both controllers
applied_delta = delta_history(1:end-1);
requested_delta = delta_command_history(1:end-1);

fprintf('\nSteering Constraint Diagnostics\n');

fprintf('Max requested steering: %.2f deg\n', ...
    rad2deg(max(abs(requested_delta))));

fprintf('Max applied steering: %.2f deg\n', ...
    rad2deg(max(abs(applied_delta))));

fprintf('Time at angle limit: %.2f %%\n', ...
    100 * mean(abs(applied_delta) >= delta_max - 1e-6));

fprintf('RMS applied steering: %.2f deg\n', ...
    rad2deg(sqrt(mean(applied_delta.^2))));

if make_plots
    figure;
    plot(t(1:end-1), rad2deg(delta_command_history(1:end-1)), ...
        '--', 'LineWidth', 1.2);
    hold on;
    plot(t(1:end-1), rad2deg(delta_history(1:end-1)), ...
        'LineWidth', 1.5);
    
    yline(rad2deg(delta_max), ':');
    yline(-rad2deg(delta_max), ':');
    
    xlabel('Time (s)');
    ylabel('Steering angle (deg)');
    title('Requested vs. Applied Steering');
    legend('Requested', 'Applied', 'Angle limits', ...
        'Location', 'best');
    grid on;
    hold off;
end

% Evaluate tracking after the initial recovery period
tracking_idx = t >= tracking_start_time;

rms_ey_tracking = sqrt(mean(ey_history(tracking_idx).^2));
max_ey_tracking = max(abs(ey_history(tracking_idx)));

rms_epsi_tracking = sqrt(mean(epsi_history(tracking_idx).^2));

fprintf('\nPost-startup Tracking Metrics (t >= %.1f s)\n', ...
    tracking_start_time);
fprintf('RMS cross-track error: %.4f m\n', rms_ey_tracking);
fprintf('Maximum cross-track error: %.4f m\n', max_ey_tracking);
fprintf('RMS heading error: %.2f deg\n', rad2deg(rms_epsi_tracking));

% Cross-track error by time interval
interval_edges = linspace(0, T, 7);
intervals = [interval_edges(1:end-1)', ...
    interval_edges(2:end)'];

fprintf('\nCross-Track Error by Time Interval\n');
for j = 1:size(intervals, 1)
    if j < size(intervals, 1)
        idx = t >= intervals(j,1) & t < intervals(j,2);
    else
        idx = t >= intervals(j,1) & t <= intervals(j,2);
    end

    fprintf('%.2f-%.2f s: RMS = %.4f m, Max = %.4f m\n', ...
        intervals(j,1), intervals(j,2), ...
        sqrt(mean(ey_history(idx).^2)), ...
        max(abs(ey_history(idx))));
end

% Error at selected timestamps
check_times = linspace(0, T, 21);

fprintf('\nCross-Track Error at Selected Times\n');
for j = 1:length(check_times)
    [~, idx] = min(abs(t - check_times(j)));

    fprintf('t = %.1f s: ey = %+.4f m\n', ...
        t(idx), ey_history(idx));
end

% Include the transition from the initial steering angle of zero
steering_rate = diff([0, applied_delta]) / dt;

fprintf('\nSteering Rate Diagnostics\n');
fprintf('Maximum absolute steering rate: %.4f rad/s\n', ...
    max(abs(steering_rate)));
fprintf('Configured steering-rate limit: %.4f rad/s\n', ...
    delta_rate_max);

% Check angle constraint
fprintf('Maximum absolute applied steering: %.4f rad\n', ...
    max(abs(delta_history)));
fprintf('Configured steering-angle limit: %.4f rad\n', ...
    delta_max);

if strcmp(controller_type, 'lqr')

    fprintf('\nFeedforward Steering Diagnostics\n');

    fprintf('Minimum reference steering: %.4f deg\n', ...
        rad2deg(min(delta_ref_history(1:end-1))));

    fprintf('Maximum reference steering: %.4f deg\n', ...
        rad2deg(max(delta_ref_history(1:end-1))));

    fprintf('RMS reference steering: %.4f deg\n', ...
        rad2deg(sqrt(mean(delta_ref_history(1:end-1).^2))));

end

% Plot state responses
if make_plots && strcmp(controller_type, 'lqr')
    figure;
    
    subplot(2,2,1);
    plot(t, x(1,:));
    title('Lateral Velocity Tracking Error');
    xlabel('Time (s)');
    ylabel('v_y - v_{y,ref} (m/s)');
    grid on;
    
    subplot(2,2,2);
    plot(t, x(2,:));
    title('Yaw Rate Tracking Error');
    xlabel('Time (s)');
    ylabel('r - r_{ref} (rad/s)');
    grid on;
    
    subplot(2,2,3);
    plot(t, x(3,:));
    title('Lateral Error');
    xlabel('Time (s)');
    ylabel('e_y (m)');
    grid on;
    
    subplot(2,2,4);
    plot(t, x(4,:));
    title('Heading Error');
    xlabel('Time (s)');
    ylabel('e_\psi (rad)');
    grid on;
end

if make_plots    
    figure;
    
    plot(x_ref, y_ref, 'r--', 'LineWidth', 2);
    hold on;
    plot(X_pos, Y_pos, 'b-', 'LineWidth', 2);
    
    plot(X_pos(1), Y_pos(1), 'go', 'MarkerFaceColor', 'g');
    plot(X_pos(end), Y_pos(end), 'ro', 'MarkerFaceColor', 'r');
    
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Vehicle Trajectory vs. Reference Path');
    legend('Reference Path', 'Vehicle Trajectory', ...
        'Start', 'End', 'Location', 'best');
    
    axis equal;
    grid on;
    hold off;
end

if make_plots && strcmp(controller_type, 'lqr')
    figure;
    plot(t(1:end-1), rad2deg(delta_ref_history(1:end-1)), ...
        'LineWidth', 1.5);
    hold on;
    plot(t(1:end-1), rad2deg(delta_history(1:end-1)), ...
        '--', 'LineWidth', 1.2);
    
    xlabel('Time (s)');
    ylabel('Steering angle (deg)');
    title('Feedforward Steering vs. Applied Steering');
    legend('Reference feedforward', 'Applied steering', ...
        'Location', 'best');
    grid on;
    hold off;
end

% Return performance metrics for automated testing
if isempty(settling_idx)
    settling_time_s = NaN;
else
    settling_time_s = t(settling_idx);
end

results_nonlinear = struct();

% Configuration and controller information
results_nonlinear.vx = vx;
results_nonlinear.vx_design = vx_design;
results_nonlinear.initial_offset = initial_offset;
results_nonlinear.Q = Q;
results_nonlinear.R = R;
results_nonlinear.K = K;
results_nonlinear.closed_loop_eigenvalues = closed_loop_eigenvalues;

% Overall tracking performance
results_nonlinear.rms_cross_track_m = rms_ey;
results_nonlinear.max_cross_track_m = max_abs_ey;
results_nonlinear.rms_heading_deg = rad2deg(rms_epsi);
results_nonlinear.max_heading_deg = rad2deg(max_abs_epsi);

% Steering effort
results_nonlinear.rms_steering_deg = rad2deg(rms_delta);
results_nonlinear.max_steering_deg = rad2deg(max_abs_delta);

% Post-startup tracking performance
results_nonlinear.post_startup_rms_cross_track_m = rms_ey_tracking;
results_nonlinear.post_startup_max_cross_track_m = max_ey_tracking;
results_nonlinear.post_startup_rms_heading_deg = rad2deg(rms_epsi_tracking);

% Actuator constraints
results_nonlinear.settling_time_s = settling_time_s;
results_nonlinear.rate_limited_pct = ...
    100 * mean(delta_rate_limited_history(1:end-1));
results_nonlinear.angle_limited_pct = ...
    100 * mean(abs(applied_delta) >= delta_max - 1e-6);

results_nonlinear.max_requested_steering_deg = ...
    rad2deg(max(abs(requested_delta)));
results_nonlinear.max_applied_steering_deg = ...
    rad2deg(max(abs(applied_delta)));
results_nonlinear.max_applied_steering_rate_rad_s = ...
    max(abs(steering_rate));

% Store simulation histories for later analysis
results_nonlinear.t = t;
results_nonlinear.t_control = t(1:end-1);

results_nonlinear.x_ref = x_ref;
results_nonlinear.y_ref = y_ref;
results_nonlinear.X_pos = X_pos;
results_nonlinear.Y_pos = Y_pos;

results_nonlinear.ey_history = ey_history;
results_nonlinear.epsi_history = epsi_history;
results_nonlinear.x = x;

results_nonlinear.delta_history = delta_history(1:end-1);
results_nonlinear.delta_command_history = ...
    delta_command_history(1:end-1);
results_nonlinear.delta_ref_history = delta_ref_history(1:end-1);
results_nonlinear.alpha_f_history = alpha_f_history;
results_nonlinear.alpha_r_history = alpha_r_history;
results_nonlinear.vy_history = vy_history;
results_nonlinear.r_history = r_history;

% Tire model configuration
results_nonlinear.mu = mu;
results_nonlinear.F_zf_N = F_zf;
results_nonlinear.F_zr_N = F_zr;

% Tire force histories
results_nonlinear.F_yf_history = F_yf_history;
results_nonlinear.F_yr_history = F_yr_history;

% Maximum predicted lateral forces
results_nonlinear.max_front_force_N = ...
    max(abs(F_yf_history));

results_nonlinear.max_rear_force_N = ...
    max(abs(F_yr_history));

results_nonlinear.delta_max_rad = delta_max;
results_nonlinear.delta_rate_max_rad_s = delta_rate_max;

results_nonlinear.dt = dt;
results_nonlinear.controller_dt = controller_dt;

results_nonlinear.controller_type = controller_type;
results_nonlinear.lookahead_m = lookahead;

% Close the main simulate_lqr function
end

function [x_closest, y_closest] = ...
    closestPathPoint(X, Y, x_samples, y_samples)

% Find the nearest sampled point
distances_squared = ...
    (x_samples - X).^2 + (y_samples - Y).^2;

[~, idx] = min(distances_squared);

% Define a local search interval around that point
left_idx = max(1, idx - 1);
right_idx = min(length(x_samples), idx + 1);

x_left = x_samples(left_idx);
x_right = x_samples(right_idx);

% Minimize squared distance to the continuous reference path
objective = @(x_query) ...
    (X - x_query).^2 + ...
    (Y - 2*sin(x_query/5)).^2;

x_closest = fminbnd(objective, x_left, x_right);
y_closest = 2*sin(x_closest/5);

end