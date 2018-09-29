%% Questao 3
clear;
clc;
close all;

%% Questao 3.1, 3.3: Modelar sistema em espaco de estados

% Definicoes (igual da q2, ver como obter este modelo!!)
m = 0.5;
M = 1;
L = 1;
g = 9.81;

syms x1 x2 x3 x4 u %m M L g;
x = [x1 x2 x3 x4];

f1 = x2;
f2 = (u-m*g*sin(x3)*cos(x3)+m*L*x4^2*sin(x3))/(M+m-m*cos(x3)^2);
f3 = x4;
f4_num = m*g*sin(x3)-(u*m*L*x4^2*sin(x3))*m*cos(x3)/(M+m);
f4_den = m*L-(m^2*L*cos(x3)^2)/(M+m);
f4 = f4_num/f4_den;

f = [f1;
     f2;
     f3;
     f4];
 
u1 = 0;
u2 = 1/((M+m)-m*(cos(x3))^2);
u3 = 0;
u4 = -1*m*cos(x3)/((M+m)*m*L-m^2*L*cos(x3)^2);

uf = [u1;
      u2;
      u3;
      u4];
  
A = subs(jacobian(f,x),[x1 x2 x3 x4],[0 0 0 0])
B = subs(uf,[x3 x4],[0 0])
C = [1 0 0 0]
D = 0

disp('Controlabilidade e Observabilidade ja verificadas na Q2');

%% Questao 3.4: 

x0 = [0 0 pi/180*25 0];
x0obs = [0; 0; pi/180*10; 0];
