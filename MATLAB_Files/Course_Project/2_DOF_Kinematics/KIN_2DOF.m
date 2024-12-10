%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%                     Robot Kinematics                %%%%
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


% START OF FORWARD KINEMATICS
% Input Forward Kinematics DH Table
syms theta1 theta2

DH = [theta1, 0, L1, 0;
      theta2, 0, L2, 0];

% Call Function FK_2DOF
FK_Fnc_Call = FK_2DOF(DH); 

% UNITS ARE OF DEGREES!!!
%t = [0 0];
t = [1.1601,77.2910];

FK_Value = matlabFunction(FK_Fnc_Call);

T = FK_Value(t(1),t(2))
% END OF FORWARD KINEMATICS 


% START OF INVERSE KINEMATICS 
% Input Inverse Kinematics Parameters
% (xc,yc,L1,L2,r)

% ENSURE THAT THE OUTPUT OF R <= L1+L2;
% THIS REPRESENTS THAT THE ROBOT CAN REACH THE POINT (XC,YC)
xc = .3;
yc = .25;
L1 = .25;
L2 = .25;
r = sqrt(xc^2+yc^2);

[IK_FinalTH1, IK_FinalTH2]  = IK_2DOF(xc,yc,L1,L2,r);
% END OF INVERSE KINEMATICS


% START OF VELOCITY KINEMATICS 
% Input Velocity Kinematics Parameters
theta1 = 15;
theta2 = 50;
omega1 = 10; 
omega2 = 10; 
L1 = .25;
L2 = .25;

% values of degrees
[VKxdot, VKydot] = VK_2DOF(theta1, theta2, omega1, omega2, L1, L2)
% END OF VELOCITY KINEMATICS