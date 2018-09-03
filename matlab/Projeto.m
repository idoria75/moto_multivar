% Parâmetros do modelo
Mv = 0.152; % Massa do veículo sem giro [kg]
Mg = 0.15; % Massa do giro [kg]
Rg = 0.095/2; % Raio do giro [m]
Ag = 0.006; % Espessura giro [m]
Av = 0.075; % Altura veículo [m]
Lv = 0.19; % Largura veículo [m]
Dg = 0.06; % Distância entre centro de massa do giro e eixo de rotação [m]
Dv = 0.045; % Distância entre centro de massa do veículo e eixo de rotação
Omega = 7200*0.10472; % Velocidade de rotação do giro, rpm*conversão = rad/sec
g = 9.81; % Gravidade [m/s^2] 
IG11 = Mg*(Rg^2)/4 + Mg*(Ag^2)/12; % Algum momento de inercia
IG33 = Mg*(Rg^2)/2;
IB11 = Mv*(Av^2+Lv^2)/12;

%% Modelo matematico
% x1 = p
% x2 = theta
% x3 = p_linha
%  u = theta_linha

syms x1 x2 x3 u; 
f1 = x3;
f2 = 0;
f3_num = (Mv*Dv+Mg*Dg)*g*sin(x1);
f3_den = IB11+Mv*Dv^2+Mg*Dg^2+IG11*cos(x2)^2+IG33*sin(x2)^2;
f3 = f3_num/f3_den;
u1 = 0;
u2 = 1;
u3_num = (-2*cos(x2)*sin(x2)*(IG33-IG11)*x3)-(Omega*cos(x2)*IG33);
u3_den = IB11+Mv*Dv^2+Mg*Dg^2+IG11*cos(x2)^2+IG33*sin(x2)^2;
u3 = u3_num/u3_den;
% x_ponto = A*x+B*u

A = [f1; f2; f3];
B = [u1; u2; u3];

