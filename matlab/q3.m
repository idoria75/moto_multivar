%% Questao 3
clear;
clc;
close all;

%% Questao 3.1: Modelar sistema em espaco de estados

% Definicoes (igual da q2, ver como obter este modelo!!)
m = 0.5;
M = 1;
L = 1;
g = 9.81;

syms x1 x2 x3 x4 u;
x = [x1 x2 x3 x4];
C0 = 0;

x0 = [0 0 pi/180*25 0];
x0obs = [0; 0; pi/180*10; 0];


% aux1 = -m*g/M;
% aux2 = (m+M)*g/(M*L);
% 
% aux3 = 1/M;
% aux4 = -1/(M*L);
% 
% A = [0 1  0   0;
%      0 0 aux1 0;
%      0 0  0   1;
%      0 0 aux2 0;]
%  
% B = [  0 ;
%      aux3;
%        0 ;                                                                                                                                                                                                                  
%      aux4]
% 
% C = [1 0 0 0];
% 
% D = zeros(1,1);
% %% Determinar o conjunto dos pontos de equilibrio
% sys = ss(A,B,C,D);
