
function delta_cmd = pure_pursuit_controller( ...
    X, Y, psi, x_closest, x_ref, y_ref, L, lookahead)

% Find the first reference point ahead of the closest path point.
target_idx = find(x_ref >= x_closest, 1, 'first');

if isempty(target_idx)
    target_idx = length(x_ref);
end

% Select the first point at least lookahead distance
% from the vehicle's current position.
while target_idx < length(x_ref) && ...
        hypot(x_ref(target_idx) - X, ...
        y_ref(target_idx) - Y) < lookahead
    target_idx = target_idx + 1;
end

% Use the final available point if the requested lookahead
% extends beyond the reference path.
target_idx = min(target_idx, length(x_ref));

target_x = x_ref(target_idx);
target_y = y_ref(target_idx);

% Angle from vehicle heading to the target point.
alpha = atan2(target_y - Y, target_x - X) - psi;

% Wrap angle to [-pi, pi].
alpha = atan2(sin(alpha), cos(alpha));

% Distance from vehicle to target point.
target_distance = hypot(target_x - X, target_y - Y);

% Pure Pursuit steering command.
delta_cmd = atan2(2 * L * sin(alpha), target_distance);

end
