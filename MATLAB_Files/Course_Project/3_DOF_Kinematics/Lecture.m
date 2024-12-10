%Forward kinematics 
clc, clear
close all
a1 = 4;
a2 = 3;
a3 = 2;
syms a1 a2 a3 t1 t2 t3

% Lotfi's way
DHTable = [t1,0,4,0;t2,0,3,0;t3,0,2,0]

[n,m] = size(DHTable);
T = eye(4);
for i = 1:n
    th = DHTable(i,1);
    d = DHTable(i,2);
    a = DHTable(i,3);
    alpha = DHTable(i,4);

    A = [cos(th), - sin(th)*cos(alpha), sin(th)*sin(alpha), a*cos(th);
        sin(th), cos(th)*cos(alpha), -cos(th)*cos(alpha), a*sin(th);
        0, sin(alpha), cos(alpha), d;
        0,0,0,1]

    T = T*A
end

T_final = simplify(T)

% As a function
% HAVE FUNCTION SAME NAME AS FILE
% function T = FK_fnc(DH)
%
%   SAME AS ABOVE CODE
%

% CALLING FUNCTION
% DHTable = [t1 0 4 0;t2 0 3 0;t3 0 2 0]
% Fk_fnc(DHTable)

%%
% my way

T3 = DH(90,90,90, 0, 4,3,2, 0);


function T3 = DH(t1,t2,t3, d, a1,a2,a3, alpha)

A1 = [cosd(t1), -sind(t1), sind(t1)*sind(alpha), a1*(cosd(t1));
    sind(t1), cosd(t1)*cosd(alpha), -cosd(t1)*sind(alpha), a1*sind(t1);
    0, sin(alpha), cos(alpha), d;
    0,0,0,1];

A2 = [cosd(t2), -sind(t2), sind(t2)*sind(alpha), a2*(cosd(t2));
    sind(t2), cosd(t2)*cosd(alpha), -cosd(t2)*sind(alpha), a2*sind(t2);
    0, sin(alpha), cos(alpha), d;
    0,0,0,1];

A3 = [cosd(t3), -sind(t3), sind(t3)*sind(alpha), a3*(cosd(t3));
    sind(t3), cosd(t3)*cosd(alpha), -cosd(t3)*sind(alpha), a3*sind(t3);
    0, sin(alpha), cos(alpha), d;
    0,0,0,1];

T3 = A1*A2*A3;

end

