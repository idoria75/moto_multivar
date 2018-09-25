%% LAB4

% Codigo utilizado no laboratorio 4 p/ o pendulo invertido
% Utilizado aqui como referencia para a questao 2 do trabalho

clear all
clc

% Parametros do pendulo invertido
M = 1; 
m = 0.5;
L = 1; 
g = 9.81;

% Matrizes do modelo de estado
A = [0 1  0              0;
     0 0 -m*g/M          0;
     0 0  0              1;
     0 0 (m + M)*g/(M*L) 0];

B = [0; 1/M; 0; -1/(M*L)];

C = [1 0 0 0];

D = zeros(1,1);

Dsim = zeros(4,1); % para obter o estado x na simulacoes

x0 = [0 0 pi/4 0]; % condicao inicial do pendulo

% Analise do sistema
eig(A)             % polos da matriz A

sys=ss(A,B,C,D);   % define modelo de estado
 
G=zpk(tf(sys))     % funcao de transferencia

MC=ctrb(sys)       % matriz de controlabilidade
svd(MC)            % valores singulares de MC, em que posto(MC) = numero de valores singulares nao-nulos
                   % Nota SVD: Mais robusto que calcular o posto da matriz,
                   % e serve para matrizes ret?ngulares.

MO=obsv(sys)       % matriz de observabilidade
svd(MO)            % valores singulares de MO, em que posto(MO) = numero de valores singulares nao-nulos


% Configuracao Controlador-Observador

K=place(A,B,[-3 -3.05 -3.1 -3.15])      % determina a matriz de ganho K para posicionar os polos de A-BK em -3 
K=place(A,B,[-2 -2.05 -2.1 -2.15])    % Nota: place nao aceita polos iguais, logo usamos polos muito proximos
%K=place(A,B,[-5 -5.05 -5.1 -5.15])

L=(place(A',C',[-6 -6.05 -6.1 -6.15]))' % determina a matriz L do observador para posicionar os polos de A-LC em -9

%x0obs = [0 0 0 0];                      % condicao inicial do observador
x0obs = [0.05 0 pi/5 0];



%%%%%%%%%%%%%%%%%%%%%%%%%
% ts = 3.0 -> polos em -2
% ts = 1.6 -> polos em -5

%% LAB 5
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
%Ka = [4.5975   3.5337   0.9908   0.4067  -1.8253 0.0778  -4.9807  -0.0386   0.1826  -0.1056;
%      0.6755   0.2034   4.1541   4.4480   0.0386 0.1659   0.1402  -1.8253   0.5435  -4.9345];   % [Ka,S,E] = lqr(Aa,Ba,eye(10),0.4*eye(2)) --> controle otimo

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
%% LAB 6
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

%x0 = [0 0 0 0];    % condicao inicial do sistema
x0 = [1 2 0.8 -0.3];

x0obs = [0 0 0 0]; % condicao inicial do observador

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
pd=-1; 
% Matriz de ganho do estado aumentado xa para posicionar os polos de Aa-BaKa em pd
Ka = place(Aa,Ba,[pd pd-0.025 pd-0.05 pd-0.075 pd-0.1 pd-0.125 pd-0.15 pd-0.175 pd-0.2 pd-0.225]);

% Verificacao
polosMF = eig(Aa-Ba*Ka)

% Matriz de ganho para o estado da planta x 
K = Ka(:,1:4);

% Matriz de ganho para o estado do modelo interno xm
Km = Ka(:,5:10);

% Determinacao da observabilidade da planta
Obs = obsv(A,C);   % Matriz de Observabilidade
vsObs = svd(Obs)   % numero da valores singulares nao-nulos = posto(Obs)

% Polo repetido desejado para o observador
pobs = 2*pd;
% Matriz de ganho para posicionar os polos de A-LC em pobs
H = place(A',C',[pobs pobs-0.025 pobs-0.05 pobs-0.075]);
L = H';

% Verificacao
polosObs = eig(A-L*C)

% Matriz de transferencia do sistema aumentado em malha-fechada com o observador (1) 
E = B;
F = zeros(2,2);
Aetil = [ A    -B*Km -B*K;
         -Bm*C  Am    zeros(6,4);
          L*C  -B*Km  A-B*K-L*C ]; 

Betil = [ E        zeros(4,2);
         -Bm*F     Bm;    
          E-L*F    zeros(4,2) ];

Ctil = [C zeros(2,6) zeros(2,4)]; 

Dtil = [F zeros(2,2)];

systil=ss(Aetil,Betil,Ctil,Dtil);

Gtil=zpk(tf(systil))                   % determina Ga(s) com Y(s) = Gtil(s)Ue(s)

MatrizGanhoEstatico = dcgain(Gtil)     % calcula Gtil(0)

%figure
%pzmap(Gtil(1,3))                       % pzmap da funcao de transferencia Gtil13(s) com Y1(s) = Gtil13(s) R1(s)

% Matriz de transferencia do sistema aumentado em malha-fechada com o observador (2) 
E = B;
F = zeros(2,2);
Aetil = [ A    -B*Km -B*K;
         -Bm*C  Am    zeros(6,4);
          L*C  -B*Km  A-B*K-L*C ]; 

Bebar = [ E        zeros(4,2);
         -Bm*F     Bm;    
          E-L*F    L         ];

Ctil = [C zeros(2,6) zeros(2,4)]; 

Dtil = [F zeros(2,2)];

sysbar=ss(Aetil,Bebar,Ctil,Dtil);

Gbar=zpk(tf(sysbar))                   % determina Ga(s) com Y(s) = Gbar(s)Ue(s)

MatrizGanhoEstatico = dcgain(Gbar)     % calcula Gbar(0)

%figure
%pzmap(Gbar(1,3))                       % pzmap da funcao de transferencia Gbar13(s) com Y1(s) = Gbar13(s) R1(s)

% Matriz de transferencia Cs do controlador resultante com o observador (2) 

Ac = [ A-B*K-L*C    -B*Km;
       zeros(6,4)    Am; ]; 

Bc = [ -L;
        Bm ];

sysCs=ss(Ac,Bc,-Ka,zeros(2,2));

Cs=zpk(tf(sysCs))           