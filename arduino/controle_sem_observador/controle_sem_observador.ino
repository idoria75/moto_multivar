/*
* Universidade Federal de Santa Catarina
* Departamento de Automação e Sistemas - CTC
*
* Código de controle sem observador / Controle Multivariável
* Autor: Alex Amadeu Cani
* Jan 2018
*/

#include <Mpu6050.h>
#include <Servo.h>

// ======== CONFIGURAÇÕES =========
// Altere os valores conforme a montagem do seu projeto

// Pinos
const int pino_servo = 4;
const int pino_giroscopio = 9;

// Parâmetros servo
const int offset_servo = 90; /* Valor, em graus, que deixa o
                                giroscópio paralelo a moto.
                                Utilizar o programa teste_servo
                                para determinar o valor.
                             */
const int servo_maximo = 130; //  Limite superior da atuação do servo
const int servo_minimo = 50;  //  Limite inferior da atuação do servo
                              //  Novamente, utilize o arquivo teste_servo
                              //  para determinar o menor e maior valor para atuar

const bool inverter_direcao_servo = false; //  Modifique este parâmetro caso
                                           //  a direção da rotação do servo precise
                                           //  ser invertida (de modo que um aumento de u
                                           //  provoque um aumento de theta).

// Parâmetros IMU
const bool EIXO_X = true; /* A IMU pode ser montada de duas formas,
                             ou seja, de modo que a posição angular
                             da moto corresponda ao eixo X da IMU ou
                             ao eixo Y.
                             Valores:
                             true = eixo de rotação da moto corresponde
                                    ao eixo X da IMU
                             false = eixo de rotação da moto corresponde
                                    ao eixo Y da IMU
                           */
                         
const float offset_posicao_x = 0;    //  Valores de offset de posição e velocidade
const float offset_velocidade_x = 0; //  dos eixos X e Y. Apenas um eixo será utilizado,
const float offset_posicao_y = 0;   //  dependendo da configuração EIXO_X acima.
const float offset_velocidade_y = 0;  //  Para determinar os valores utilize o programa calibracao_imu

const bool inverter_imu = false; //  Análogo ao inverter_direcao_servo. Ajustar de modo que o sentido de rotação
                                 //  positivo da IMU seja o sentido positivo de rho
                                 
// Parâmetros de Controle
// Os estados do sistema, x1, x2 e x3 são realimentados
// através da lei de controle u = -K1*x1 + -K2*x2 + -K3*x3
// x1 -> Posição angular 'rho'
// x2 -> Posição angular do giroscópio 'theta'
// x3 -> Velocidade angular 'rho_ponto'
const float K1 = 0.0;
const float K2 = 0.0;
const float K3 = 0.0;

const int Ts = 5;  // Período de amostragem, em ms.
                   // Menor valor aconselhado 5ms

// Opções de visualização
const bool plotar_referencia = true;
const bool plotar_x1 = true;
const bool plotar_x2 = true;
const bool plotar_x3 = true;
const bool plotar_atuacao_min_max = false;

//  ======= FIM DAS CONFIGURAÇÕES =========
//  Não é necessário editar nada abaixo desta linha

Mpu6050 imu;
Servo atuador;
Servo giroscopio;
unsigned long tempo_ultimo_controle;
float x1, x2, x3;
float du, dt;
float theta = 0;
float u;
int atuacao;

const float kGrausParaRadianos = M_PI/180.0;
const float kRadianosParaGraus = 180.0/M_PI;

void setup() {
  Serial.begin(115200);
  if(!imu.Begin()) {
    Serial.println("Erro ao configurar IMU");
    Serial.end();
    while(true);
  }

  imu.SetXPosOffset(offset_posicao_x);
  imu.SetYPosOffset(offset_posicao_y); 
  imu.SetXSpeedOffset(offset_velocidade_x);
  imu.SetYSpeedOffset(offset_velocidade_y);

  atuador.attach(pino_servo);
  atuador.write(offset_servo); //  Vai para a posição inicial
  
  giroscopio.attach(pino_giroscopio); //  Sequência de inicializaçao do giro
  giroscopio.writeMicroseconds(500);
  delay(3000);
  giroscopio.writeMicroseconds(1000);
  delay(3000);
  giroscopio.writeMicroseconds(2000);

  delay(12000); //  Espera o giroscópio acelerar para iniciar o controle

  tempo_ultimo_controle = millis();
}

void loop() {
  //  IMU deve ser atualizada o mais frequente possível devido aos filtros.
  imu.Update();
  dt = millis() - tempo_ultimo_controle;
  
  if (dt >= Ts) { // Loop de controle
    Rotation rot = imu.GetRotation();
    Velocity vel = imu.GetVelocity();
    
    x1 = EIXO_X ? rot.x : rot.y;
    x1 *= (inverter_imu ? -1 : 1);
    x1 *= kGrausParaRadianos;
    
    x2 = theta;
  
    x3 = EIXO_X ? vel.x : vel.y;
    x3 *= (inverter_imu ? -1 : 1);
    x3 *= kGrausParaRadianos;

    u = -K1*x1 -K2*x2 -K3*x3;

    theta += (u*dt)/1000.0; //  Converter dt para segundos
    // Atua
    atuacao = static_cast<int>((inverter_direcao_servo ? -1 : 1)*kRadianosParaGraus*theta + offset_servo);
    if (atuacao > servo_maximo) {
      atuacao = servo_maximo;
      theta = kGrausParaRadianos*(servo_maximo-offset_servo); //  Não pode deixar theta ficar crescendo!
    } else if (atuacao < servo_minimo) {
      atuacao = servo_minimo;
      theta = kGrausParaRadianos*(servo_minimo-offset_servo);
    }
    
    atuador.write(atuacao);
    tempo_ultimo_controle = millis();
    
    // Plots
    if (plotar_referencia) {
      Serial.print(0);
      Serial.print('\t');
    }
    if (plotar_x1) {
      Serial.print(x1);
      Serial.print('\t');
    }
    if (plotar_x2) {
      Serial.print(x2);
      Serial.print('\t');
    }
    if (plotar_x3) {
      Serial.print(x3);
      Serial.print('\t');
    }
    if (plotar_atuacao_min_max) {
      Serial.print(kGrausParaRadianos*servo_maximo);
      Serial.print('\t');
      Serial.print(kGrausParaRadianos*servo_minimo);
      Serial.print('\t');
    }
    Serial.println();
  }
}
