%%%% Adaptive PID Controller (APID)
%%%% Process: Gp(s) = exp(-0.2s)/(s^2+2s+1)

clc; clear all;

% Simulation parameters
h = 0.1;
t = 0:h:16;
tf = length(t);

% Initialize arrays
y = zeros(1, length(t));
u = zeros(1, length(t));
e = zeros(1, length(t));
x = zeros(1, length(t));
kpp = zeros(1, length(t));      % Adaptive proportional gain
kii = zeros(1, length(t));      % Adaptive integral gain
kdd = zeros(1, length(t));      % Adaptive derivative gain
v = zeros(1, length(t));        % Gain updating factor

% Initial conditions
r = 1;
y(1) = 0;
x(1) = 0;
e(1) = r - y(1);

% Ziegler-Nichols base parameters
ku = 10.5;
tu = 2.033;

kp_base = 0.6 * ku;
ti = tu / 2;
td = tu / 8;
ki_base = kp_base * (h / ti);
kd_base = kp_base * (td / h);

% APID tuning parameters (from thesis)
k1 = 1;
k2 = 1;
k3 = 12;
k4 = 0.3;                        % Base integral gain factor

fprintf('APID Parameters:\n');
fprintf('kp_base = %.4f\n', kp_base);
fprintf('ki_base = %.4f\n', ki_base);
fprintf('kd_base = %.4f\n', kd_base);
fprintf('k1 = %.1f, k2 = %.1f, k3 = %.1f, k4 = %.1f\n', k1, k2, k3, k4);

% Initial control
u(1) = kp_base * e(1) + ki_base * sum(e);
kpp(1) = kp_base;
kii(1) = ki_base;
kdd(1) = kd_base;
v(1) = 0;

% System dynamics for first 2 steps
F_xy = @(x) -2*x;

for i = 1:2
    % RK4
    k_1 = F_xy(x(i));
    k_2 = F_xy(x(i) + 0.5*h*k_1);
    k_3 = F_xy(x(i) + 0.5*h*k_2);
    k_4 = F_xy(x(i) + k_3*h);
    
    x(i+1) = x(i) + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4)*h;
    y(i+1) = y(i) + h*x(i+1);
    e(i+1) = r - y(i+1);
    
    % Calculate gain updating factor
    ed = e(i+1) - e(i);
    v(i+1) = (e(i+1)/r) * (ed/r);
    
    % Update adaptive gains
    kpp(i+1) = kp_base * (1 + k1 * abs(v(i+1)));
    kii(i+1) = ki_base * (k4 + k2 * v(i+1));
    kdd(i+1) = kd_base * (1 + k3 * abs(v(i+1)));
    
    % Adaptive control
    u(i+1) = kpp(i+1)*e(i+1) + kii(i+1)*sum(e) + kdd(i+1)*ed;
end

% Main simulation loop
F_xy = @(u, y, x) u - y - 2*x;

% Calculate midpoint
mid_point = floor(tf/2);

% Setpoint tracking phase
for i = 3:mid_point
    % RK4
    k_1 = F_xy(u(i-2), y(i), x(i));
    k_2 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_1);
    k_3 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_2);
    k_4 = F_xy(u(i-2) + h, y(i) + h, x(i) + k_3*h);
    
    x(i+1) = x(i) + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4)*h;
    y(i+1) = y(i) + h*x(i+1);
    e(i+1) = r - y(i+1);
    
    % Update adaptive gains
    ed = e(i+1) - e(i);
    v(i+1) = (e(i+1)/r) * (ed/r);
    kpp(i+1) = kp_base * (1 + k1 * abs(v(i+1)));
    kii(i+1) = ki_base * (k4 + k2 * v(i+1));
    kdd(i+1) = kd_base * (1 + k3 * abs(v(i+1)));
    
    % Adaptive control
    u(i+1) = kpp(i+1)*e(i+1) + kii(i+1)*sum(e) + kdd(i+1)*ed;
end

% Apply load disturbance
u(mid_point + 1) = -14;
fprintf('\nLoad disturbance applied at index %d (t = %.1f sec)\n', mid_point + 1, mid_point*h);

% Load disturbance rejection phase
for i = (mid_point + 1):(length(t)-1)
    % RK4
    k_1 = F_xy(u(i-2), y(i), x(i));
    k_2 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_1);
    k_3 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_2);
    k_4 = F_xy(u(i-2) + h, y(i) + h, x(i) + k_3*h);
    
    x(i+1) = x(i) + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4)*h;
    y(i+1) = y(i) + h*x(i+1);
    e(i+1) = r - y(i+1);
    
    % Update adaptive gains
    ed = e(i+1) - e(i);
    v(i+1) = (e(i+1)/r) * (ed/r);
    kpp(i+1) = kp_base * (1 + k1 * abs(v(i+1)));
    kii(i+1) = ki_base * (k4 + k2 * v(i+1));
    kdd(i+1) = kd_base * (1 + k3 * abs(v(i+1)));
    
    % Adaptive control
    u(i+1) = kpp(i+1)*e(i+1) + kii(i+1)*sum(e) + kdd(i+1)*ed;
end

% Plot results
figure(1);
plot(t, y, 'b-', 'LineWidth', 2);
hold on;
plot(t, r*ones(size(t)), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Output y', 'FontSize', 12);
title('APID Response: Gp(s) = exp(-0.2s)/(s^2+2s+1)', 'FontSize', 12);
legend('Process Output', 'Setpoint', 'Location', 'southeast');
grid on;

% Calculate performance metrics
y_max = max(y(1:mid_point));
overshoot = (y_max - r) * 100;

% Rise time
tr_idx = find(y >= 0.99*r, 1);
if ~isempty(tr_idx)
    rise_time = (tr_idx - 1) * h;
else
    rise_time = NaN;
end

% Settling time
settling_idx = mid_point;
for i = mid_point:-1:1
    if abs(y(i) - r) > 0.02*r
        settling_idx = i;
        break;
    end
end
settling_time = (settling_idx - 1) * h;

% IAE and ITAE
IAE = h * sum(abs(e));
ITAE = h * sum(t .* abs(e));

% Display results
fprintf('\n--- APID Performance Metrics ---\n');
fprintf('Overshoot: %.2f%%\n', overshoot);
fprintf('Rise Time: %.2f s\n', rise_time);
fprintf('Settling Time: %.2f s\n', settling_time);
fprintf('IAE: %.4f\n', IAE);
fprintf('ITAE: %.4f\n', ITAE);