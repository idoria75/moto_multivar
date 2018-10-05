clc
clear all
close all
%%
% Par?metros do modelo
Mv = 0.152; % Massa do ve?culo sem giro [kg]
Mg = 0.15; % Massa do giro [kg]
Rg = 0.095/2; % Raio do giro [m]
Ag = 0.006; % Espessura giro [m]
Av = 0.075; % Altura ve?culo [m]
Lv = 0.19; % Largura ve?culo [m]
Dg = 0.06; % Dist?ncia entre centro de massa do giro e eixo de rota??o [m]
Dv = 0.045; % Dist?ncia entre centro de massa do ve?culo e eixo de rota??o
Omega = 7200*0.10472; % Velocidade de rota??o do giro, rpm*convers?o = rad/sec
g = 9.81; % Gravidade [m/s^2] 
IG11 = Mg*(Rg^2)/4 + Mg*(Ag^2)/12; % Algum momento de inercia
IG33 = Mg*(Rg^2)/2;
IB11 = Mv*(Av^2+Lv^2)/12; 

%% modelo

syms x1 x2 x3 u;
f1 = x3;
f2 = 0;
f3 = ((Mv*Dv+Mg*Dg)*g*sin(x1))/(IB11+Mv*(Dv^2)+IG11*(cos(x2)^2)+Mg*(Dg^2)+IG33*((sin(x2))^2));
f = [f1;f2;f3];
u1 = 0;
u2 = 1;
u3 = (-2*cos(x2)*sin(x2)*x3*(IG33-IG11)-Omega*cos(x2)*IG33)/(IB11+IG11*(cos(x2)^2)+Mv*(Dv^2)+Mg*(Dg^2)+IG33*(sin(x2)^2));
u=[u1;u2;u3];

A = double(subs(jacobian(f),[x1 x2],[0 0]))
B = double(subs(u, [x1 x2],[0 0]));
C = [1 1 0];
D = zeros(3,1);
x0 = [pi/4 0 0];
P1 = [-12.01 -12.02 -12];
K = place(A,B,P1);

%% Controle realimenta??o de estados

% Verifica a controlabilidade do Sistema Aumentado
Con = ctrb(A,B)
vsCon = rank(Con)

r = 0.7;
R = r;
Q = eye(3);
Ka = lqr(A, B, Q, R)

polos = eig(A-B*Ka)

%% Observador


% Verifica a observabilidade da Planta
Obs = obsv(A,C)
vsObs = rank(Obs)

x0obs = [0 ; 0; 0];  

pd = -10;
L=place(A',C',[pd pd-0.05 pd-0.03])';        % polo duplo de A-LC em s=-12     

%% Ruido

V1 = 0.001*eye(3);
V2 = 0.0005;

Lkf = lqr(A',C',V1,V2);
Lkf = Lkf'