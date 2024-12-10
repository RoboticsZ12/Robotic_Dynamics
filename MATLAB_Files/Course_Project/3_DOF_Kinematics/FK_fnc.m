function T_final = FK_fnc(DH)

[n,m] = size(DH);
T = eye(4);

for i = 1:n
    th = DH(i,1);
    d = DH(i,2);
    a = DH(i,3);
    alpha = DH(i,4);

    A = [cos(th) -sin(th)*cos(alpha) sin(th)*sin(alpha) a*cos(th);
        sin(th) cos(th)*cos(alpha) -cos(th)*sin(alpha) a*sin(th);
        0 sin(alpha) cos(alpha) d;
        0 0 0 1];
    T = T*A;
end

T_final = simplify(T);
