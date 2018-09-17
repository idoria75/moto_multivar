%% Questao #2
clear;
clc;
close all;

% Definicoes
m = 0.5;
M = 1;
L = 1;
g = 9.81;

aux1 = -m*g/M;
aux2 = (m+M)*g/(M*L);

aux3 = 1/M;
aux4 = -1/(M*L);

A = [0 1  0   0;
     0 0 aux1 0;
     0 0  0   1;
     0 0 aux2 0;]
 
B = [  0 ;
     aux3;
       0 ;
     aux4]

C = [1 0 0 0];

D = zeros(1,1);
% Modelo valido para theta aprox. 0 e theta_ponto aprox. 0

% Numero de linhas em A (numero de estados do sistema)
n_numEstados = size(A,1);  % x1, x2, x3, x4
m_numEntradas = 1;         % Somente u eh entrada
p_numSaidas = 1;           % Somente x1 eh saida

%% 2.1: Determine estabilidade interna da origem
disp('------Item 1------')

% Estabilidade interna eh verificada
% Para isso, todos os autovalores de A, que sao polos de G(s)
% devem estar no SPE.
disp('Autovalores de A:')
autovalores_A = eig(A);
disp(autovalores_A);
disp('Matriz A possui autovalores no SPD!');
disp('Matriz A nao eh est?vel internamente');
%Pergunta: E se tiver autovalor em 0?
%Resposta: Nada podemos afirmar.

%% 2.2.a: Verificar Controlabilidade e Observabilidade
disp('------Item 2------')
% Verificar Controlabilidade
% MC = [B A*B A^2*B ... A^(n-1)*B]
MC = [B A*B A^2*B A^3*B];

% OBS: svd() eh mais robusto do que calcular o posto da matriz.
% Resultado de SVD eh igual ao posto!

% Calcula os valores nao-singulares da matriz MC.
% A matriz eh controlavel se o numero de valores nao-nulos do
% posto de uma matriz eh igual ao numero de estados!

disp('Valores singulares de MC: ');
disp(svd(MC));

% Verificar controlabilidade
% MO = [C; C*A; C*A^2; ...; C*A^(n-1)]
MO = [  C   ;
       C*A  ;
      C*A^2 ;
      C*A^3];

% Calcula os valores nao-singulares da matriz MO
disp('Valores singulares de MO: ');
disp(svd(MO));
disp('');
disp('Posto(MC) = 4: Matriz eh Controlavel');
disp('Posto(MO) = 4: Matriz eh Observavel ');

% % Outro metodo: 
% sys=ss(A,B,C,D);
% G=zpk(minreal(tf(sys)))
% MC=ctrb(sys)
% svd(MC)    
% MO=obsv(sys)
% svd(MO)

%% Questao 2.2.b: Observabilidade quando y = x3?
C_2b = [0 0 1 0];

MO_2b = [  C_2b   ;
          C_2b*A  ;
         C_2b*A^2 ;
         C_2b*A^3];

disp(' ');
disp('Valores singulares de MO_2b: ');
disp(svd(MO_2b));

%% Questao 2.3: Projeto de Controlador com Observador de Estados (Simulink)

% Define o sistema trabalhado:
sys = ss(A,B,C,D);
G=zpk(minreal(tf(sys)));

% Condicao inicial do sistema
x0 = [0 0 0 0]; 

Dsim = zeros(4,1);

%% Questao 2.4: Projeto de Controlador sem Observador de Estados (Simulink)

% Modelo Interno
% Beta(s) = S

% Matrizes formadas a partir de beta (ver pg. 75 notas de aula)
M = [0];
N = [1];
Am = M;
Bm = N;
Cm = eye(1);
Dm = zeros(1,1);

% Sistema aumentado
Aa = [A zeros(4,1); -Bm*C Am];
Ba = [B ;
      0];

% Verificacao da controlabilidade do sistema aumentado
MC_SistAum = [Ba Aa*Ba Aa^2*Ba Aa^3*Ba Aa^4*Ba];
disp('Valores singulares de MC_SistAum:');
disp(svd(MC_SistAum));
disp('Todos os valores sao maiores do que zero.');
disp('Sistema aumentado eh controlavel');

% Polos desejados
pd = -4.4;
disp('Polos desejados');
disp(pd);
Ka = place(Aa,Ba,[pd pd-0.05 pd-0.1 pd-0.15 pd-0.2]);
%disp('Ka:');
%disp(Ka);

% Verificacao
disp('Polos MF:');
polosMF = eig(Aa-Ba*Ka);

% Matriz de ganho para o estado da planta x
disp('Ganho p/ estado da planta');
K = Ka(:,1:4);
disp(K);
% Matriz de ganho para o estado do modelo interno xm
disp('Ganho para estado aumentado');
Km = Ka(5);
disp(Km);

%% Questao 2.5: Projeto de Controlador com Observador de Estados p/ 
%               rejeicao de perturbacoes senoidais (referencia nula).

%% Executa Simulink
simOut = sim('q2_model');