%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%                    Robotic Simulation               %%%%
%%%%              Programmer: Zechariah Georgian         %%%%
%%%%         Class: MRE 454 (Robotic Dynamic Control)    %%%%
%%%%                    Due: 12/06/2024                  %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc, clear

% Parameters
% ALL PARAMETERS ARE IN UNITS OF METRIC SYSTEM (kg, m)
L1 = .25;
L2 = .25;
r1 = .125;
r2 = .125;
m1 = .5;
m2 = .5;
g = 9.81;
I1 = (m1*L1^2)/12;
I2 = (m2*L2^2)/12;
b1 = 10^(-1); 
b2 = 10^(-1);
tau = [0; 0];  
% END OF PARAMETERS

% START OF SIMULATION
% M(theta)*theta(..)+C(theta,theta(.))*theta(.)+g(theta)=tau
% SOLVE FOR THETA(..)
% Theta(..) = (tau - g(theta) - C(theta,theta(.))*theta(.))/(M(theta))

% theta(1) = Joint angle 1
% theta(2) = Joint angle 2
% theta(3) = Angular velocity of Joint angle 1
% theta(4) = Angular velocith of Joint angle 2
 f = @(t, theta) [
    theta(3);
    theta(4);
    
    % Inertia matrix M(theta)
    [I1 + I2 + m1*r1^2 + m2*(L1^2 + r2^2) + 2*m2*L1*r2*cosd(theta(2)), I2 + m2*r2^2 + m2*L1*r2*cosd(theta(2));
     I2 + m2*r2^2 + m2*L1*r2*cosd(theta(2)), I2 + m2*r2^2] \ [ -theta(3)*(-m2*L1*r2*sind(theta(2))*theta(4) + b1) - m2*L1*r2*sind(theta(2))*(theta(3) + theta(4)) - ((m1*r1 + m2*L1)*g*cosd(theta(1)) + m2*r2*g*cosd(theta(1) + theta(2))) + tau(1);
        -theta(3)*(m2*L1*r2*sind(theta(2))*theta(3) + b2) - m2*r2*g*cosd(theta(1) + theta(2)) + tau(2)]
];

dt = .01;
tspan = 0:dt:10;
initial_cond = [90; -90; 0; 0]; % Initial conditions: [theta1, theta2, theta1_dot, theta2_dot]
[ts,thetas] = ode45(f,tspan,initial_cond)

figure;
subplot(2,1,1);
plot(ts, thetas(:, 1), 'b-', 'LineWidth', 1.5); 
hold on;
plot(ts, thetas(:, 2), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Joint Angles (Degrees)');
legend('\theta_1 Angle', '\theta_2 Angle');
title('Theta 1/2 vs. Time');
grid on

% If there was a third joint, would continue adding joints. We only have
% two, so we are good. 
x_EndEffector = L1*cosd(thetas(:,1))+L2*cosd(thetas(:,1)+thetas(:,2));
y_EndEffector = L1*sind(thetas(:,1))+L2*sind(thetas(:,1)+thetas(:,2));

% End Effector position with reference to joint angles. 
subplot(2,1,2);
plot(x_EndEffector, y_EndEffector, 'r-', 'LineWidth', 1.5);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title("End Effector Position W/Reference to Joint Angles");
grid on
% END OF SIMULATION
