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
f2 = (-m*g*sin(x3)*cos(x3)+m*L*x4^2*sin(x3))/(M+m-m*cos(x3)^2);
f3 = x4;
f4_num = m*g*sin(x3)-((u+m*L*x4^2*sin(x3))*m*cos(x3))/(M+m);
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
  
A = double(subs(jacobian(f,x),[x1 x2 x3 x4],[0 0 0 0]))
B = double(subs(uf,[x3 x4],[0 0]))
C = [1 0 0 0]
D = zeros(2,2)
Dsim = zeros(4,1);

disp('Controlabilidade e Observabilidade ja verificadas na Q2');

%% Questao 3.4: (Igual a Q2)

C0 = 0;
x0 = [0 0 pi/180*25 0];
x0Obs = [0; 0; pi/180*10; 0];

% Matrizes formadas a partir de beta (ver pg. 75 notas de aula)
M = [0];
N = [1];
Am = M;
Bm = N;
Cm = eye(1);
Dm = zeros(1,1);

% Sistema aumentado
Aa = [A zeros(4,1); -Bm*C Am];
%Aa = double(Aa);
Ba = [B ;
      0];
%Ba = double(Ba);

% Por posicionamento de Polos:
pd = -3;
disp('Polos desejados');
disp(pd);
Ka = place(Aa,Ba,[pd pd-0.025 pd-0.05 pd-0.075 pd-0.1]);

% Matriz de ganho para o estado da planta x
disp('Ganho p/ estado da planta');
K = Ka(:,1:4);
disp(K);
% Matriz de ganho para o estado do modelo interno xm
disp('Ganho para estado aumentado');
Km = Ka(5);
disp(Km);

% Polos do observador mais rapidos que os adicionados ao sistema
pdObs = 2*pd;
% Posiciona polos do observador
H = place(A', C',[pdObs-.025, pdObs-0.05, pdObs-0.075, pdObs-0.1]);
L = H';

% Beta = s^2+0*s+0.1^2 -> alpha0 = 0.1^2=0.01; alpha1 = 0
alpha0 = 0;
alpha1 = -0.01;
alpha2 = 0;

M_q5 = [zeros(2,1) eye(2)    ;
        alpha0 alpha1 alpha2];

N_q5 = [0 ;
        0 ;
        1];
    
Am_q5 = M_q5;
Bm_q5 = N_q5

Cm_q5 = eye(3);
Dm_q5 = zeros(3,1);

Aa_q5 = [   A     zeros(4,3) ;
         -Bm_q5*C   Am_q5   ];

Ba_q5 = [B ;
         0 ;
         0 ;
         0];
     
MC_q5 = [Ba_q5 Aa_q5*Ba_q5 Aa_q5^2*Ba_q5 Aa_q5^3*Ba_q5 Aa_q5^4*Ba_q5 Aa_q5^6*Ba_q5 Aa_q5^7*Ba_q5];

disp('Valores singulares de MC_SistAum_q5:');
format long;
disp(svd(MC_q5));
format short;
disp('Todos os valores sao maiores do que zero.');
disp('Sistema aumentado eh controlavel');

% Polos desejados
pd_q5 = -2.5;
disp('Polos desejados');
disp(pd_q5);
Ka_q5 = place(Aa_q5,Ba_q5,[pd_q5 pd_q5-0.025 pd_q5-0.05 pd_q5-0.075 pd_q5-0.1 pd_q5-0.125 pd_q5-0.150]);
%disp('Ka:');
%disp(Ka);

% Verificacao
disp('Polos MF:');
polosMF_q5 = eig(Aa_q5-Ba_q5*Ka_q5);
disp(polosMF_q5);

% Matriz de ganho para o estado da planta x
disp('Ganho p/ estado da planta');
K_q5 = Ka_q5(:,1:4);
disp(K_q5);
% Matriz de ganho para o estado do modelo interno xm
disp('Ganho para estado aumentado');
Km_q5 = Ka_q5(5:7);
disp(Km_q5);

%% Q3.4: LQR

Q = eye(5);
r = 10^-3;
R = eye(1)*r
Ka_lqr = lqr(Aa,Ba,Q,R);
Kx_lqr = Ka_lqr(1,1:4)
Km_lqr = Ka_lqr(1,5)
eig(Aa-Ka*Ba)
%% Q3.5: LQG