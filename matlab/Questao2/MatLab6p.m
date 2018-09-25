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

