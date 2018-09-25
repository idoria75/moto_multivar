% Sistema Massa-Mola

clear all;
clc;

% Planta
m1=1; m2=2;
k1=1; k2=1;

A = [     0        1       0        0; 
     -(k1 + k2)/m1 0     k2/k1      0;
          0        0       0        1;
        k2/m2      0  -(k1 + k2)/m1 0];
   
B = [ 0    0;
     1/m1  0;
      0    0;
      0   1/m2];

C = [1 0 0 0;
     0 0 1 0];

D = zeros(2,2);

Dsim = zeros(4,2); % para obter o estado x na simulacoes

x0 = [1 0.5 1 0.1];% condicao inicial do sistema

% Analise do sistema
eig(A)             % polos da matriz A

sys=ss(A,B,C,D);   % define modelo de estado
 
G=zpk(tf(sys))     % matriz de transferencia

% Modelo Interno
coef = [0 -1 0];   % coeficientes de beta

M = [zeros(2,1) eye(2); coef];

N = [0; 0 ; 1];

Am = blkdiag(M,M);
Bm = blkdiag(N,N);

% Sistema Aumentado
Aa = [ A     zeros(4,6);
      -Bm*C    Am        ];

Ba = [B ; zeros(6,2) ];

% Determinacao da controlabilidade do sistema aumentado
Con = ctrb(Aa,Ba);  % Matriz de Controlabilidade
vsCon = svd(Con)    % numero da valores singulares nao-nulos = posto(Con)

% Polo repetido desejado
pd=-1.0 
% Matriz de ganho do estado aumentado xa para posicionar os polos de Aa-BaKa em pd
Ka = place(Aa,Ba,[pd pd-0.025 pd-0.05 pd-0.075 pd-0.1 pd-0.125 pd-0.15 pd-0.175 pd-0.2 pd-0.225]);

% Questao 3:
Ka = [4.5975   3.5337   0.9908   0.4067  -1.8253 0.0778  -4.9807  -0.0386   0.1826  -0.1056;
      0.6755   0.2034   4.1541   4.4480   0.0386 0.1659   0.1402  -1.8253   0.5435  -4.9345];   % [Ka,S,E] = lqr(Aa,Ba,eye(10),0.4*eye(2)) --> controle otimo

% Verificacao
polosMF = eig(Aa-Ba*Ka)

% Matriz de ganho para o estado da planta x 
K = Ka(:,1:4);

% Matriz de ganho para o estado do modelo interno xm
Km = Ka(:,5:10);

% Matriz de transferencia do sistema aumentado em malha-fechada
E = B;
F = zeros(2,2);
Ae = Aa-Ba*Ka;
Be = [   E  zeros(4,2);
      -Bm*F     Bm    ];
Ca = [C zeros(2,6)]; 
Da = [F zeros(2,2)];

sysa=ss(Ae,Be,Ca,Da);

Ga=zpk(tf(sysa))                       % determina Ga(s) com Y(s) = Ga(s)Ue(s)

MatrizGanhoEstatico = dcgain(Ga)       % calcula Ga(0)

%figure
%pzmap(Ga(1,1))                         % pzmap da funcao de transferencia Ga11(s) com Y1(s) = Ga11(s) W1(s)

%figure
%pzmap(Ga(1,3))                         % pzmap da funcao de transferencia Ga13(s) com Y1(s) = Ga13(s) R1(s)




%%%%%%%%%%%%%%%%%
% ts=10 --> pd=-1