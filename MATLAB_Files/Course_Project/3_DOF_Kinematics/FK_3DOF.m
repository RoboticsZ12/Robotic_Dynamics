clc, clear 
close all

syms t1 t2 t3

DH = [t1 0 4 0;t2 0 3 0;t3 0 2 0];

sym_FK = FK_fnc(DH);

t = [0 0 0];
t = [10 20 30]*pi/180;
t = [90 90 90]*pi/180;

num_FK = matlabFunction(sym_FK);

T = num_FK(t(1),t(2),t(3))
