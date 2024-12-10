clc, clear
close all

Tx = @(a)[1 0 0 a;0 1 0 0;0 0 1 0;0 0 0 1];
Ty = @(b)[1 0 0 0;0 1 0 b;0 0 1 0;0 0 0 1];
Tz = @(c)[1 0 0 0;0 1 0 0;0 0 1 c;0 0 0 1];

Rotx =@(alpha)[1 0 0 0;0 cosd(alpha) -sind(alpha) 0;0 sind(alpha) cosd(alpha) 0;0 0 0 1];
Roty =@(beta)[cosd(beta) 0 sind(beta) 0;0 1 0 0;-sind(beta) 0 cosd(beta) 0;0 0 0 1];
Rotz =@(gamma)[cosd(gamma) -sind(gamma) 0 0;sind(gamma) cosd(gamma) 0 0;0 0 1 0;0 0 0 1];
