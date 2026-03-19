%%%% Genetic Algorithm based PID Controller (GA-CPID)
%%%% Process: Gp(s) = exp(-0.2s)/(s+1)^2
%%%% Optimizes: Kp, Ki, Kd

clc; clear all;

% ==================== GA PARAMETERS ====================
popsize = 10;                    % Population size
Nt = 12;                          % Total bits (4 bits × 3 variables)
no_of_variable = 3;
nmut = 2;                          % Number of mutations per generation
max_gen = 50;                      % Maximum generations

% Objective function weights
w1 = 1;                            % IAE weight
w2 = 0;                             % ITAE weight (0 for IAE only)

% ==================== PROCESS PARAMETERS ====================
h = 0.1;
t = 0:h:16;
tf = length(t);
mid_point = floor(tf/2);           % Calculate midpoint for load disturbance

% Ziegler-Nichols base parameters
ku = 10.5;
tu = 2.0333;

kp_zn = 0.6 * ku;
ti = tu / 2;
td = tu / 8;
ki_zn = kp_zn * (h / ti);
kd_zn = kp_zn * (td / h);

fprintf('Ziegler-Nichols Base Values:\n');
fprintf('Kp = %.4f\n', kp_zn);
fprintf('Ki = %.4f\n', ki_zn);
fprintf('Kd = %.4f\n\n', kd_zn);

% Parameter bounds (±20% of ZN values)
xh1 = kp_zn + 0.20 * kp_zn;
xl1 = kp_zn - 0.20 * kp_zn;

xh2 = ki_zn + 0.20 * ki_zn;
xl2 = ki_zn - 0.20 * ki_zn;

xh3 = kd_zn + 0.20 * kd_zn;
xl3 = kd_zn - 0.20 * kd_zn;

fprintf('Search Ranges:\n');
fprintf('Kp: [%.4f, %.4f]\n', xl1, xh1);
fprintf('Ki: [%.4f, %.4f]\n', xl2, xh2);
fprintf('Kd: [%.4f, %.4f]\n\n', xl3, xh3);

% ==================== INITIALIZE POPULATION ====================
% Create random binary population
pop = round(rand(popsize, Nt));

% Split into 3 variables (4 bits each)
aa = pop(:, 1:4);
bb = pop(:, 5:8);
cc = pop(:, 9:12);

% Convert binary to decimal
d1 = bi2de(aa, 'left-msb');
d2 = bi2de(bb, 'left-msb');
d3 = bi2de(cc, 'left-msb');

% Convert to actual parameter values
for i = 1:popsize
    x1(i, 1) = xl1 + ((xh1 - xl1) / (2^4 - 1)) * d1(i);
    x2(i, 1) = xl2 + ((xh2 - xl2) / (2^4 - 1)) * d2(i);
    x3(i, 1) = xl3 + ((xh3 - xl3) / (2^4 - 1)) * d3(i);
end

d = [x1, x2, x3];

% ==================== EVALUATE INITIAL POPULATION ====================
fprintf('Evaluating initial population...\n');
iae = zeros(popsize, 1);

for j = 1:popsize
    kp = d(j, 1);
    ki = d(j, 2);
    kd = d(j, 3);
    
    % Simulate system - get both IAE and output
    [IAE, ~] = simulate_pid(kp, ki, kd, h, t, tf, mid_point);
    iae(j) = IAE;
end

% Sort by fitness (lower IAE is better)
[iae, ind] = sort(iae);
pop = pop(ind, :);
d = d(ind, :);

fprintf('Best initial IAE = %.4f\n', iae(1));

% ==================== GA MAIN LOOP ====================
fprintf('\nStarting GA optimization...\n');
fprintf('Generation\tBest IAE\tMean IAE\n');

for gen = 1:max_gen
    % ========== SELECTION AND CROSSOVER ==========
    % Select top 5 parents and create offspring
    p1 = [pop(1,1:2) pop(2,3:4) pop(1,5:6) pop(2,7:8) pop(1,9:10) pop(2,11:12)];
    p2 = [pop(2,1:2) pop(1,3:4) pop(2,5:6) pop(1,7:8) pop(2,9:10) pop(1,11:12)];
    p3 = [pop(3,1:2) pop(4,3:4) pop(3,5:6) pop(4,7:8) pop(3,9:10) pop(4,11:12)];
    p4 = [pop(4,1:2) pop(3,3:4) pop(4,5:6) pop(3,7:8) pop(4,9:10) pop(3,11:12)];
    p5 = [pop(5,1:2) pop(1,3:4) pop(5,5:6) pop(1,7:8) pop(5,9:10) pop(1,11:12)];
    
    m = [p1; p2; p3; p4; p5];
    pop(6:10, :) = m;
    
    % ========== DECODE POPULATION ==========
    aa = pop(:, 1:4);
    bb = pop(:, 5:8);
    cc = pop(:, 9:12);
    
    d1 = bi2de(aa, 'left-msb');
    d2 = bi2de(bb, 'left-msb');
    d3 = bi2de(cc, 'left-msb');
    
    for i = 1:popsize
        x1(i, 1) = xl1 + ((xh1 - xl1) / (2^4 - 1)) * d1(i);
        x2(i, 1) = xl2 + ((xh2 - xl2) / (2^4 - 1)) * d2(i);
        x3(i, 1) = xl3 + ((xh3 - xl3) / (2^4 - 1)) * d3(i);
    end
    
    d = [x1, x2, x3];
    
    % ========== EVALUATE FITNESS ==========
    for j = 1:popsize
        kp = d(j, 1);
        ki = d(j, 2);
        kd = d(j, 3);
        
        [IAE, ~] = simulate_pid(kp, ki, kd, h, t, tf, mid_point);
        iae(j) = IAE;
    end
    
    % ========== SORT POPULATION ==========
    [iae, ind] = sort(iae);
    pop = pop(ind, :);
    d = d(ind, :);
    
    % ========== MUTATION ==========
    mrow = ceil(rand(1, nmut) * (popsize - 1)) + 1;
    mcol = ceil(rand(1, nmut) * Nt);
    
    for ii = 1:nmut
        pop(mrow(ii), mcol(ii)) = abs(pop(mrow(ii), mcol(ii)) - 1);
    end
    
    % Display progress every 10 generations
    if mod(gen, 10) == 0 || gen == 1
        fprintf('%d\t\t%.4f\t\t%.4f\n', gen, iae(1), mean(iae));
    end
end

% ==================== FINAL RESULTS ====================
fprintf('\n========== OPTIMIZATION COMPLETE ==========\n');
fprintf('Best Solution:\n');
fprintf('Kp = %.4f\n', d(1, 1));
fprintf('Ki = %.4f\n', d(1, 2));
fprintf('Kd = %.4f\n', d(1, 3));
fprintf('Best IAE = %.4f\n', iae(1));

% ==================== SIMULATE BEST SOLUTION ====================
% Now get both output and error for performance metrics
[IAE_best, y, e] = simulate_pid_full(d(1,1), d(1,2), d(1,3), h, t, tf, mid_point);

% ==================== PERFORMANCE METRICS ====================
y_max = max(y(1:mid_point));
overshoot = (y_max - 1) * 100;

tr_idx = find(y >= 0.99, 1);
if ~isempty(tr_idx)
    rise_time = (tr_idx - 1) * h;
else
    rise_time = NaN;
end

settling_idx = mid_point;
for i = mid_point:-1:1
    if abs(y(i) - 1) > 0.02
        settling_idx = i;
        break;
    end
end
settling_time = (settling_idx - 1) * h;

% Use the error array from simulation
IAE_final = h * sum(abs(e));
ITAE_final = h * sum(t .* abs(e));

fprintf('\n--- Performance Metrics ---\n');
fprintf('Overshoot: %.2f%%\n', overshoot);
fprintf('Rise Time: %.2f s\n', rise_time);
fprintf('Settling Time: %.2f s\n', settling_time);
fprintf('IAE: %.4f\n', IAE_final);
fprintf('ITAE: %.4f\n', ITAE_final);

% Plot the final response
figure(2);
plot(t, y, 'b-', 'LineWidth', 2);
hold on;
plot(t, ones(size(t)), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Output y', 'FontSize', 12);
title('GA-CPID Optimized Response', 'FontSize', 12);
legend('Process Output', 'Setpoint', 'Location', 'southeast');
grid on;

% ==================== HELPER FUNCTIONS ====================

% Function for GA fitness evaluation (returns only IAE)
function [IAE, y] = simulate_pid(kp, ki, kd, h, t, tf, mid_point)
    % Initialize
    y = zeros(1, length(t));
    u = zeros(1, length(t));
    e = zeros(1, length(t));
    x = zeros(1, length(t));
    
    r = 1;
    y(1) = 0;
    x(1) = 0;
    e(1) = r - y(1);
    
    u(1) = kp * e(1) + ki * sum(e);
    
    % First 2 steps
    F_xy = @(x) -2*x;
    
    for i = 1:2
        k_1 = F_xy(x(i));
        k_2 = F_xy(x(i) + 0.5*h*k_1);
        k_3 = F_xy(x(i) + 0.5*h*k_2);
        k_4 = F_xy(x(i) + k_3*h);
        
        x(i+1) = x(i) + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4)*h;
        y(i+1) = y(i) + h*x(i+1);
        e(i+1) = r - y(i+1);
        
        ed = e(i+1) - e(i);
        u(i+1) = kp*e(i+1) + ki*sum(e) + kd*ed;
    end
    
    % Main simulation
    F_xy = @(u, y, x) u - y - 2*x;
    
    % Setpoint tracking
    for i = 3:mid_point
        k_1 = F_xy(u(i-2), y(i), x(i));
        k_2 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_1);
        k_3 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_2);
        k_4 = F_xy(u(i-2) + h, y(i) + h, x(i) + k_3*h);
        
        x(i+1) = x(i) + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4)*h;
        y(i+1) = y(i) + h*x(i+1);
        e(i+1) = r - y(i+1);
        
        ed = e(i+1) - e(i);
        u(i+1) = kp*e(i+1) + ki*sum(e) + kd*ed;
    end
    
    % Apply load disturbance
    u(mid_point + 1) = -20;
    
    % Load disturbance rejection
    for i = (mid_point + 1):(length(t)-1)
        k_1 = F_xy(u(i-2), y(i), x(i));
        k_2 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_1);
        k_3 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_2);
        k_4 = F_xy(u(i-2) + h, y(i) + h, x(i) + k_3*h);
        
        x(i+1) = x(i) + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4)*h;
        y(i+1) = y(i) + h*x(i+1);
        e(i+1) = r - y(i+1);
        
        ed = e(i+1) - e(i);
        u(i+1) = kp*e(i+1) + ki*sum(e) + kd*ed;
    end
    
    % Calculate IAE
    IAE = h * sum(abs(e));
end

% Full simulation function for final evaluation (returns y and e)
function [IAE, y, e] = simulate_pid_full(kp, ki, kd, h, t, tf, mid_point)
    % Initialize
    y = zeros(1, length(t));
    u = zeros(1, length(t));
    e = zeros(1, length(t));
    x = zeros(1, length(t));
    
    r = 1;
    y(1) = 0;
    x(1) = 0;
    e(1) = r - y(1);
    
    u(1) = kp * e(1) + ki * sum(e);
    
    % First 2 steps
    F_xy = @(x) -2*x;
    
    for i = 1:2
        k_1 = F_xy(x(i));
        k_2 = F_xy(x(i) + 0.5*h*k_1);
        k_3 = F_xy(x(i) + 0.5*h*k_2);
        k_4 = F_xy(x(i) + k_3*h);
        
        x(i+1) = x(i) + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4)*h;
        y(i+1) = y(i) + h*x(i+1);
        e(i+1) = r - y(i+1);
        
        ed = e(i+1) - e(i);
        u(i+1) = kp*e(i+1) + ki*sum(e) + kd*ed;
    end
    
    % Main simulation
    F_xy = @(u, y, x) u - y - 2*x;
    
    % Setpoint tracking
    for i = 3:mid_point
        k_1 = F_xy(u(i-2), y(i), x(i));
        k_2 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_1);
        k_3 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_2);
        k_4 = F_xy(u(i-2) + h, y(i) + h, x(i) + k_3*h);
        
        x(i+1) = x(i) + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4)*h;
        y(i+1) = y(i) + h*x(i+1);
        e(i+1) = r - y(i+1);
        
        ed = e(i+1) - e(i);
        u(i+1) = kp*e(i+1) + ki*sum(e) + kd*ed;
    end
    
    % Apply load disturbance
    u(mid_point + 1) = -20;
    
    % Load disturbance rejection
    for i = (mid_point + 1):(length(t)-1)
        k_1 = F_xy(u(i-2), y(i), x(i));
        k_2 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_1);
        k_3 = F_xy(u(i-2) + 0.5*h, y(i) + 0.5*h, x(i) + 0.5*h*k_2);
        k_4 = F_xy(u(i-2) + h, y(i) + h, x(i) + k_3*h);
        
        x(i+1) = x(i) + (1/6)*(k_1 + 2*k_2 + 2*k_3 + k_4)*h;
        y(i+1) = y(i) + h*x(i+1);
        e(i+1) = r - y(i+1);
        
        ed = e(i+1) - e(i);
        u(i+1) = kp*e(i+1) + ki*sum(e) + kd*ed;
    end
    
    % Calculate IAE
    IAE = h * sum(abs(e));
    
    % Plot
    figure;
    plot(t, y, 'b-', 'LineWidth', 2);
    hold on;
    plot(t, r*ones(size(t)), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Output y');
    title(sprintf('PID Response: Kp=%.3f, Ki=%.3f, Kd=%.3f', kp, ki, kd));
    legend('Output', 'Setpoint');
    grid on;
end