%%%% Conventional PID Controller
%%%% Process: Gp(s) = exp(-0.2s)/(s^2+2s+1)

clc; clear all;

% Simulation parameters
h = 0.1;                        % Step size
t = 0:h:16;                     % Time vector (0 to 16 sec)
tf = length(t);                 % Number of time steps

% Initialize arrays
y = zeros(1, length(t));        % Process output
u = zeros(1, length(t));        % Controller output
e = zeros(1, length(t));        % Error
x = zeros(1, length(t));        % State variable

% Initial conditions
r = 1;                          % Setpoint
y(1) = 0;
x(1) = 0;
e(1) = r - y(1);

% Ziegler-Nichols tuning parameters
ku = 10.5;                      % Ultimate gain
tu = 2.033;                      % Ultimate period

% Calculate PID parameters
kc = 0.6 * ku;                   % Proportional gain
ti = tu / 2;                     % Integral time
td = tu / 8;                      % Derivative time

% Display tuning parameters
fprintf('Ziegler-Nichols PID Parameters:\n');
fprintf('Kc = %.4f\n', kc);
fprintf('Ti = %.4f\n', ti);
fprintf('Td = %.4f\n', td);

% Initial control action
u(1) = kc * (e(1) + (h/ti) * sum(e));

% System dynamics for first 2 steps (first-order part)
F_xy = @(x) -2*x;

for i = 1:2
    % RK4 method
    k_1 = F_xy(x(i));
    k_2 = F_xy(x(i) + 0.5*h*k_1);
    k_3 = F_xy(x(i) + 0.5*h*k_2);
    k_4 = F_xy(x(i) + k_3*h);
    
    x(i+1) = x(i) + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4)*h;
    y(i+1) = y(i) + h*x(i+1);
    e(i+1) = r - y(i+1);
    
    % PID control
    ed = e(i+1) - e(i);
    u(i+1) = kc * (e(i+1) + (h/ti)*sum(e) + (td/h)*ed);
end

% Main simulation loop (second-order part)
F_xy = @(u, y, x) u - y - 2*x;

% Calculate midpoint index (for load disturbance)
mid_point = floor(tf/2);        % Use floor to ensure integer

% First half: Setpoint tracking
for i = 3:mid_point
    % RK4
    k_1 = F_xy(u(i-2), y(i), x(i));
    k_2 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_1);
    k_3 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_2);
    k_4 = F_xy(u(i-2) + h, y(i) + h, x(i) + k_3*h);
    
    x(i+1) = x(i) + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4)*h;
    y(i+1) = y(i) + h*x(i+1);
    e(i+1) = r - y(i+1);
    
    % PID control
    ed = e(i+1) - e(i);
    u(i+1) = kc * (e(i+1) + (h/ti)*sum(e) + (td/h)*ed);
end

% Apply load disturbance at t = 8 sec (mid_point + 1)
u(mid_point + 1) = -14;
fprintf('\nLoad disturbance applied at index %d (t = %.1f sec)\n', mid_point + 1, (mid_point)*h);

% Second half: Load disturbance rejection
for i = (mid_point + 1):(length(t)-1)
    % RK4
    k_1 = F_xy(u(i-2), y(i), x(i));
    k_2 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_1);
    k_3 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_2);
    k_4 = F_xy(u(i-2) + h, y(i) + h, x(i) + k_3*h);
    
    x(i+1) = x(i) + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4)*h;
    y(i+1) = y(i) + h*x(i+1);
    e(i+1) = r - y(i+1);
    
    % PID control
    ed = e(i+1) - e(i);
    u(i+1) = kc * (e(i+1) + (h/ti)*sum(e) + (td/h)*ed);
end

% Plot results
figure(1);
plot(t, y, 'b-', 'LineWidth', 2);
hold on;
plot(t, r*ones(size(t)), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Output y', 'FontSize', 12);
title('CPID Response: Gp(s) = exp(-0.2s)/(s^2+2s+1)', 'FontSize', 12);
legend('Process Output', 'Setpoint', 'Location', 'southeast');
grid on;

% Calculate performance metrics
y_max = max(y(1:mid_point));
overshoot = (y_max - r) * 100;

% Rise time (0 to 100%)
tr_idx = find(y >= 0.99*r, 1);
if ~isempty(tr_idx)
    rise_time = (tr_idx - 1) * h;
else
    rise_time = NaN;
end

% Settling time (within ±2%)
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
fprintf('\n--- Performance Metrics ---\n');
fprintf('Overshoot: %.2f%%\n', overshoot);
fprintf('Rise Time: %.2f s\n', rise_time);
fprintf('Settling Time: %.2f s\n', settling_time);
fprintf('IAE: %.4f\n', IAE);
fprintf('ITAE: %.4f\n', ITAE);