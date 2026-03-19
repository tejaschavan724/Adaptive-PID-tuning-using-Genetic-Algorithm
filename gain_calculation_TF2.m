%%%% calculation of ultimate gain and time period 
%%%% tf = exp(-0.2s)/(s(s+1))

clc; clear all;

h = 0.1;
t = 0:h:30;
y = zeros(1,length(t));
u = zeros(1,length(t));
e = zeros(1,length(t));
x = zeros(1,length(t));

r = 1;                          % Setpoint
y(1) = 0;
x(1) = 0;
e(1) = r - y(1);

% Ultimate gain for this process
kp = 5.09;                       % Found by trial

u(1) = kp * e(1);

% First-order dynamics
F_xy = @(x) -x;

for i = 1:2
    k_1 = F_xy(x(i));
    k_2 = F_xy(x(i) + 0.5*h*k_1);
    k_3 = F_xy(x(i) + 0.5*h*k_2);
    k_4 = F_xy(x(i) + k_3*h);
    
    x(i+1) = x(i) + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4)*h;
    y(i+1) = y(i) + h*x(i+1);
    e(i+1) = 1 - y(i+1);
    u(i+1) = kp * e(i+1);
end

% System dynamics
F_xy = @(u,x) u - x;

for i = 3:length(t)
    k_1 = F_xy(u(i-2), x(i));
    k_2 = F_xy(u(i-2) + 0.5*h, x(i) + 0.5*h*k_1);
    k_3 = F_xy(u(i-2) + 0.5*h, x(i) + 0.5*h*k_2);
    k_4 = F_xy(u(i-2) + h, x(i) + k_3*h);
    
    x(i+1) = x(i) + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4)*h;
    y(i+1) = y(i) + h*x(i+1);
    e(i+1) = 1 - y(i+1);
    u(i+1) = kp * e(i+1);
end

z = y(1:301);
plot(t, z, 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Output y', 'FontSize', 12);
title('Ultimate Gain Test: k_p = 5.09, Ultimate Period ≈ 2.9 sec', 'FontSize', 12);
grid on;

fprintf('Ultimate Gain (Ku) = %.2f\n', kp);