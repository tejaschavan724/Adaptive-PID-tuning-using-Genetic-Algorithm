%%%% calculation of ultimate gain and time period 
%%%% tf = exp(-0.2s)/(s^2+2s+1)

clc; clear all;

h = 0.1;                       % Step size
t = 0:h:30;                     % Time vector
y = zeros(1,length(t));         % Output
u = zeros(1,length(t));         % Controller output
e = zeros(1,length(t));         % Error
x = zeros(1,length(t));         % State variable

input = 1;                      % Setpoint
y(1) = 0;
x(1) = 0;
e(1) = input - y(1);

% Try different kp values until sustained oscillation occurs
kp = 10.5;                       % Ultimate gain found by trial

u(1) = kp * e(1);

% First-order system dynamics (for first 2 steps)
F_xy = @(x) -2*x; 

for i = 1:2
    % RK4 method
    k_1 = F_xy(x(i));
    k_2 = F_xy(x(i) + 0.5*h*k_1);
    k_3 = F_xy(x(i) + 0.5*h*k_2);
    k_4 = F_xy(x(i) + k_3*h);
    
    x(i+1) = x(i) + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4)*h;
    y(i+1) = y(i) + h*x(i+1);
    e(i+1) = 1 - y(i+1);
    u(i+1) = kp * e(i+1);
end

% Second-order system dynamics
F_xy = @(u,y,x) u - y - 2*x;

for i = 3:length(t)
    k_1 = F_xy(u(i-2), y(i), x(i));
    k_2 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_1);
    k_3 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_2);
    k_4 = F_xy(u(i-2) + h, y(i) + h, x(i) + k_3*h);
    
    x(i+1) = x(i) + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4)*h;
    y(i+1) = y(i) + h*x(i+1);
    e(i+1) = 1 - y(i+1);
    u(i+1) = kp * e(i+1);
end

z = y(1:301);
plot(t, z, 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Output y', 'FontSize', 12);
title('Ultimate Gain Test: k_p = 10.5, Ultimate Period = 2.033 sec', 'FontSize', 12);
grid on;

% Display results
fprintf('Ultimate Gain (Ku) = %.2f\n', kp);

% Calculate ultimate period from the plot
% Find peaks in the last part of response
[peaks, locs] = findpeaks(z(200:end));
if length(locs) >= 2
    Tu = (locs(2) - locs(1)) * h;
    fprintf('Ultimate Period (Tu) = %.4f sec\n', Tu);
end