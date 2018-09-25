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
                   % e serve para matrizes retângulares.

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