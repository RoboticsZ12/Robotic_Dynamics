function [IK_FinalTH1, IK_FinalTH2] = IK_2DOF(xc,yc,L1,L2,r)
% IN_KIN: Inverse Kinematics of 2DOF Robot (Double Prismatic Joint Robot)
% xc = User inputted position for the X coordinate
% yc = User inputted position for the y coordinate
% L1 = Length of the first link
% L2 = Length of the second Link

T2C = (r^2-L1^2-L2^2)/(2*L1*L2);
T2S = sqrt(1-T2C^2);
TH2 = atan2(T2S,T2C)*180/pi; % Convert atan2() from radians to degrees
TH2rad = atan2(T2S,T2C);

gamma = atan2(yc,xc); % Entire angle from cos(0) to current angle location
phi = atan2(L2*sin(TH2rad),L1+L2*cos(TH2rad)); % Angle within the oblique triangle

TH1 = (gamma - phi)*180/pi;

IK_FinalTH1 = TH1
IK_FinalTH2 = TH2
end
