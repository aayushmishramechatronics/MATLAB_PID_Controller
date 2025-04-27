clc;
clear;
close all;

% Given transfer function parameters
% G(s) = 0.02 / [(3.3 + 0.00025s)(0.0005s + 0.0001) + 0.02*0.0]

% First, let's expand the denominator polynomial
% (3.3 + 0.00025s)(0.0005s + 0.0001) = 
% 3.3*0.0005s + 3.3*0.0001 + 0.00025s*0.0005s + 0.00025s*0.0001
% = 0.00165s + 0.00033 + 0.000000125s^2 + 0.000000025s
% = 0.000000125s^2 + (0.00165 + 0.000000025)s + 0.00033
% ≈ 0.000000125s^2 + 0.00165s + 0.00033

% So the transfer function becomes:
% G(s) = 0.02 / (0.000000125s^2 + 0.00165s + 0.00033)

% Let's rewrite it in standard form:
numerator = 0.02;
denominator = [0.000000125, 0.00165, 0.00033];

% Create the transfer function
G = tf(numerator, denominator);

% Display the transfer function
disp('Original Plant Transfer Function:');
G;

% Step 1: Analyze the open-loop system
figure;
subplot(2,1,1);
step(G);
title('Open-Loop Step Response');
grid on;

subplot(2,1,2);
impulse(G);
title('Open-Loop Impulse Response');
grid on;

% Step 2: Check stability of open-loop system
open_loop_poles = pole(G);
if all(real(open_loop_poles) < 0)
    disp('Open-loop system is stable');
else
    disp('Open-loop system is unstable');
end

% Step 3: Design a controller to achieve 50 rad/sec steady-state speed
% We'll use a PI controller to eliminate steady-state error
% Target: omega_ss = 50 rad/sec

% Calculate required DC gain
DC_gain = dcgain(G);
Kp_initial = 50 / DC_gain; % Initial proportional gain estimate

% Let's try a PI controller with Kp = Kp_initial and Ki = 0.1*Kp
Kp = Kp_initial;
Ki = 0.1 * Kp;
controller = tf([Kp Ki], [1 0]);

% Create closed-loop system
closed_loop = feedback(controller*G, 1);

% Step 4: Analyze the closed-loop system
figure;
step(closed_loop);
title('Closed-Loop Step Response with PI Controller');
grid on;

% Check steady-state value
ss_value = dcgain(closed_loop) * 1; % Step input of 1V
disp(['Steady-state value with initial PI controller: ', num2str(ss_value), ' rad/sec']);

% If not 50 rad/sec, adjust gains
if abs(ss_value - 50) > 0.1
    % Need to adjust controller gains
    % Since it's a PI controller, the steady-state error should be zero
    % for step input, so the issue might be with the initial gain selection
    
    % Let's try a different approach - use root locus to select gains
    figure;
    rlocus(controller*G);
    title('Root Locus with PI Controller');
    grid on;
    
    % Interactively select a point on the root locus that gives good performance
    % [K, poles] = rlocfind(controller*G);
    
    % For this example, we'll manually adjust the gains
    % Let's increase Kp to get faster response while maintaining stability
    Kp = Kp * 2;
    Ki = Kp * 0.2; % Keep the ratio but increase both
    controller = tf([Kp Ki], [1 0]);
    closed_loop = feedback(controller*G, 1);
    
    figure;
    step(closed_loop);
    title('Adjusted Closed-Loop Step Response');
    grid on;
    
    ss_value = dcgain(closed_loop) * 1;
    disp(['Adjusted steady-state value: ', num2str(ss_value), ' rad/sec']);
end

% Step 5: Verify stability of closed-loop system
closed_loop_poles = pole(closed_loop)
if all(real(closed_loop_poles) < 0)
    disp('Closed-loop system is stable');
else
    disp('Closed-loop system is unstable');
end

% Step 6: Detailed root locus analysis
figure;
rlocus(controller*G);
title('Root Locus of Controller * Plant');
grid on;

% Step 7: Frequency response analysis
figure;
bode(closed_loop);
title('Closed-Loop Bode Plot');
grid on;

% Step 8: Display final controller and performance metrics
disp('Final Controller:');
controller

% Calculate performance metrics
step_info = stepinfo(closed_loop);
disp('Performance Metrics:');
disp(['Rise Time: ', num2str(step_info.RiseTime), ' seconds']);
disp(['Settling Time: ', num2str(step_info.SettlingTime), ' seconds']);
disp(['Overshoot: ', num2str(step_info.Overshoot), '%']);

% Step 9: Simulate response to 50 rad/sec command
t = 0:0.001:1; % Time vector
u = 50 * ones(size(t)); % Step command for 50 rad/sec
figure;
lsim(closed_loop, u, t);
title('Response to 50 rad/sec Command');
xlabel('Time (s)');
ylabel('Angular Speed (rad/sec)');
grid on;
