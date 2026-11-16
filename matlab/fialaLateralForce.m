
function F_y = fialaLateralForce(alpha, C_alpha, F_z, mu)

% Validate inputs
if ~isscalar(alpha) || ~isfinite(alpha)
    error('Slip angle must be a finite scalar.');
end

if C_alpha <= 0 || F_z <= 0 || mu <= 0
    error('Cornering stiffness, normal load, and mu must be positive.');
end

% Normalized slip parameter
s = C_alpha * tan(alpha) / (mu * F_z);

% Fiala brush tire model
if abs(s) < 3

    F_y = mu * F_z * ...
        (s - abs(s)*s/3 + s^3/27);

else

    % Fully saturated tire
    F_y = mu * F_z * sign(s);

end

end
