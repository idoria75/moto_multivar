/*
 * Calibração da IMU
 * Autor: Alex Cani
 */

 #include <Mpu6050.h>

/*
 * Este código serve para determinar os valores de offset
 * para calibração da IMU.
 * 
 * Utilize o Monitor Serial ou o Plotter Serial para determinar
 * os valores que devem ser SOMADOS às variáveis de modo a zerá-las.
 */
 Mpu6050 imu;

// Itere sobre estes valores e quando estiver correto anote-os e use-os
// no código de controle.
// Mais opções de offset estão disponíveis no código da biblioteca Mpu6050
// Na prática, apenas a velocidade e posição em um dos eixos será utilizada.
// Este código permite alterar os offsets tanto em x quanto em y prevendo o 
// uso da IMU rotacionada em 90º no chassi do veículo.
// Identifique qual é o eixo correto (dada a construção do veículo e as marcações na IMU
// e preocupe-se apenas com este.

//float offset_posicao_x = 0.0;
float offset_posicao_x = 0.65;

//float offset_posicao_y = 0.0;
float offset_posicao_y = 0.0;

//float offset_velocidade_x = 0.0;
float offset_velocidade_x = -10.58;

//float offset_velocidade_y = 0.0;
float offset_velocidade_y = 0.0;

void setup() {
  Serial.begin(115200); // Inicia serial em baud rate 115200
  imu.Begin();

  // Aplica os offsets
  imu.SetXPosOffset(offset_posicao_x);
  imu.SetYPosOffset(offset_posicao_y); 
  imu.SetXSpeedOffset(offset_velocidade_x);
  imu.SetYSpeedOffset(offset_velocidade_y);
}

void loop() {
  imu.Update(); // Pega os valores atuais

  Rotation rot = imu.GetRotation(); // Pega rotação e velocidade
  Velocity vel = imu.GetVelocity();

  // Imprime os valores, comente a linha para desabilitar
  Serial.print("X: ");
  Serial.print(rot.x); Serial.print('\t');
  Serial.print(vel.x); Serial.print('\t');

  Serial.print("Y: ");
  Serial.print(rot.y); Serial.print('\t');
  Serial.print(vel.y); Serial.print('\t');

  Serial.println();

  delay(10);
}
