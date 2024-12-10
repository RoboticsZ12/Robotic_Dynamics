function [xdot, ydot] = VK_2DOF(theta1, theta2, omega1, omega2, L1, L2)    
% VK_2DOF: Velocity kinematics of 2DOF planar robot. 
% GIVENS: Theta1, Theta2, omega1, omega2, L1, and L2
% GOAL: Provide the angular velocities of end effector given variables
% above.

% L1/L2 are the lengths of the robot arms
% Omega1/Omega2 represent the velocities of each joint 
% Theta1/Theta2 represent the joint angles
% This function has also been programmed to output in terms of degrees.

    J = [-L1*sind(theta1)-L2*sind(theta1+theta2), -L2*sind(theta1+theta2);
          L1*cosd(theta1)+L2*cosd(theta1+theta2), L2*cosd(theta1+theta2);
          1,1];
    
    % Angular velocities of joint 1 (omega1) and joint 2 (omega2)
    theta_dot = [omega1; 
                 omega2]; 
    
    EE_velocity = J * theta_dot;
    
    % Extract x and y components of the end-effector velocity
    xdot = EE_velocity(1);
    ydot = EE_velocity(2);
end
