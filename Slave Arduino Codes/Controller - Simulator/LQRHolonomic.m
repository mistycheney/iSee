%%Edoardo Cacciavillani%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%LQR controller in SS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Mi Piace La Figa%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Holonomic LQR Parameter generator%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
wm=5;       %cost of torque, high to save energy%
alpha=0.7;
R=0.155;    %m%
Ra=1/(R*alpha);
wxy=5;      %penalty on position%
wv=0.1;     %penalty on speed%
wb=1;       %penalty on orientation%
wbdot=1;    %penalty on angular rate%
%%MANCA LA MASSA!!!!%%

A=[zeros(3,3),eye(3,3);zeros(3,6)];
B=[zeros(3,4);0 -1 0 1;1 0 -1 0;Ra Ra Ra Ra];
Q= [wxy 0 0 0 0 0;
    0 wxy 0 0 0 0;
    0 0 wb 0 0 0;
    0 0 0 wv 0 0;
    0 0 0 0 wv 0;
    0 0 0 0 0 wbdot];
R=eye(4,4)*wm;

[K,S,e] = lqr(A,B,Q,R);

