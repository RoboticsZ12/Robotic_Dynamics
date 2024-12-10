function T_final = FK_2DOF(DH)
% Forward Kinemtics Function
% This is basically the code we used from class, but has 
% been changed to work with a 2DOF robot vs a 3DOF. 
% The code has also been changed to convert everything into 
% units of degrees

[n,m] = size(DH);
T = eye(4);

for i = 1:n
    th = DH(i,1);
    d = DH(i,2);
    a = DH(i,3);
    alpha = DH(i,4);

    A = [cosd(th) -sind(th)*cosd(alpha) sind(th)*sind(alpha) a*cosd(th);
        sind(th) cosd(th)*cosd(alpha) -cosd(th)*sind(alpha) a*sind(th);
        0 sind(alpha) cosd(alpha) d;
        0 0 0 1];
    T = T*A;
end

T_final = simplify(T);
