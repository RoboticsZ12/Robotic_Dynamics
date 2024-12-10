%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%                    PD Robot Controller              %%%%
%%%%              Programmer: Zechariah Georgian         %%%%
%%%%         Class: MRE 454 (Robotic Dynamic Control)    %%%%
%%%%                    Due: 12/06/2024                  %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc, clear  

% Robot Specs (GIVEN)
L1 = 0.25;
L2 = 0.25;  
r1 = 0.125; 
r2 = 0.125; 
m1 = 0.5; 
m2 = 0.5;    
g = 9.81;              
I1 = m1 * L1^2 / 12;   
I2 = m2 * L2^2 / 12;   
b1 = 0.1;
b2 = 0.1;
% END OF ROBOT SPECS

% Declaring plotting time (GIVEN)
dt = 0.01;                
t = 0:dt:5; 

% EE desired location (LOCATION .3 GIVEN) 
x_d = 0.3;
y_d = 0.3;
D = (x_d^2 + y_d^2 - L1^2 - L2^2) / (2 * L1 * L2);
theta2_d = atan2(sqrt(1 - D^2), D);
theta1_d = atan2(y_d, x_d) - atan2(L2 * sin(theta2_d), L1 + L2 * cos(theta2_d));
theta_d = [theta1_d; theta2_d];

%Initial Joint angle conditions (GIVEN)
theta = [pi/2; 0];    
theta_dot = [0; 0];  

% TEST VALUES
%Kp = [3,.1];
%Kd = [.02,.05];

%controller gains (Gives slight overshoot, with stable system)
Kp = [4,.2];
Kd = [.02,.05];

%unstable condition
%Kp = [5,.5];
%Kd = [.5,.05];

% Data storage for plotting
theta_history = zeros(2, length(t));
tau_history = zeros(2, length(t));
xy_history = zeros(2, length(t));

% Simulation loop
for i = 1:length(t)
    
    e = theta - theta_d;
    e_dot = theta_dot;
    
    % Gravity vectors
    theta1 = theta(1);
    theta2 = theta(2);

    g_theta1 = m1 * g * r1 * cos(theta1) + m2 * g * (L1 * cos(theta1) + r2 * cos(theta1 + theta2));
    g_theta2 = m2 * g * r2 * cos(theta1 + theta2);
    g_theta = [g_theta1; g_theta2];
    
    tau = -Kp * e - Kd * e_dot + g_theta;
    
    % For some reason declaring as a normal matrix was throwing an error. 
    % When researching the issue, the solution was using "diag".
    B = diag([I1, I2]);          
    C = diag([b1, b2]);           

    % Euler equation for velocities
    theta_ddot = B \ (tau - C * theta_dot); 
    
    % joint status update 
    theta_dot = theta_dot + theta_ddot * dt;
    theta = theta + theta_dot * dt;
    
    % Forward kinematics
    x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
    
    % Store data for later plotting!!!
    theta_history(:, i) = theta;
    tau_history(:, i) = tau;
    xy_history(:, i) = [x; y];
end

% Plotting
% Plotting Joint Angles
figure;
subplot(3,1,1);
plot(t, theta_history(1,:), 'm', t, theta_history(2,:), 'c');
title('Joint Angles');
xlabel('Time [s]');
ylabel('Angle [rad]');
legend('\theta_1', '\theta_2');
grid on

% Plotting Torque 
subplot(3,1,2);
plot(t, tau_history(1,:), 'm', t, tau_history(2,:), 'c');
title('Joint Torques');
xlabel('Time [s]');
ylabel('Torque [Nm]');
legend('\tau_1', '\tau_2');
grid on

% Plotting EE Position
subplot(3,1,3);
plot(xy_history(1,:), xy_history(2,:), 'b');
title('End-Effector Position');
xlabel('X [m]');
ylabel('Y [m]');
axis equal;
grid on
