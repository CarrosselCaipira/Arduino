#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define ODOMETRIA_ESQ A0
#define ODOMETRIA_DIR A1
#define TESTE_ODOMETRIA_ESQ A2
#define TESTE_ODOMETRIA_DIR A3
byte contOdoEsq = 0;//contador de pulsos, zerado a cada inicio de contagem (a cada novo comando)
byte contOdoDir = 0;
char ultContagemOdoEsq = 0;//ultima contagem desde a ultima medida
char ultContagemOdoDir = 0;
int valMinEsq = 1000;
int valMinDir = 1000;
int valMaxEsq = 0;
int valMaxDir = 0;
unsigned long tempoAnterior = 0;

int numero = 0;
int erro = 0;

// Define qual robo é esse (0, 1 ou 2)
int robo = 0;

#define PWM_MOTOR_ESQ 5
#define DIRECAO_PWM_MOTOR_ESQ_A 6
#define DIRECAO_PWM_MOTOR_ESQ_B 7
#define DIRECAO_PWM_MOTOR_DIR_A 8
#define DIRECAO_PWM_MOTOR_DIR_B 9
#define PWM_MOTOR_DIR 10

RF24 radio(2, 3);

// Armazena os bytes recebidos pelo rádio
byte text[9];

boolean dirE = false;
boolean dirD = false;
boolean got = false;

const byte rxAddr[6] = { 'U', 'n', 'e', 's', 'p' };

void setup()
{
  pinMode(PWM_MOTOR_ESQ, OUTPUT);
  pinMode(DIRECAO_PWM_MOTOR_ESQ_A, OUTPUT);
  pinMode(DIRECAO_PWM_MOTOR_ESQ_B, OUTPUT);
  pinMode(DIRECAO_PWM_MOTOR_DIR_A, OUTPUT);
  pinMode(DIRECAO_PWM_MOTOR_DIR_B, OUTPUT);
  pinMode(PWM_MOTOR_DIR, OUTPUT);
  pinMode(ODOMETRIA_ESQ, INPUT);
  pinMode(ODOMETRIA_DIR, INPUT);
  
  while (!Serial);
  Serial.begin(9600);
  
  radio.begin();
  radio.openReadingPipe(0, rxAddr);
  
  radio.startListening();
}

void loop()
{
  got = false;
  com();
  if(got) {
    anda();
  }
  
  
  //verificaOdoEsq();
  //verificaOdoDir();
}

// Recebe comandos novos
void com() {
  if (radio.available())
  {
    got = true;
    radio.read(&text, sizeof(text));
    if(text[0] == 0x80){
      Serial.println("Recebemos algo");
    }
  }
}

// Movimenta os motores
void anda() {

    dirE = HIGH;
    dirD = HIGH;

// Atribui as velocidades

  /*
    // Trabalha os valores deslocados (0 - 256)
    // Verifica a direção da roda esquerda
    if(text[1+2*robo] >= 128) {
      dirE = LOW;
      // Tira o primeiro bit
      text[1+2*robo] = text[1+2*robo] - 128;
    }
    else {
      dirE = HIGH;
    }

    // Verifica a direção da roda direita
    if(text[2+2*robo] >= 128) {
      dirD = LOW;
      text[2+2*robo] = text[2+2*robo] - 128;
    }
    else {
      text[2+2*robo] = text[2+2*robo] - 128;
    }
   */

   // Verifica a direção da roda esquerda
    if(text[1+2*robo] < 0) {
      dirE = LOW;
      // Tira o primeiro bit
      text[1+2*robo] = text[1+2*robo] * -1;
    }
    else {
      dirE = HIGH;
    }

    // Verifica a direção da roda direita
    if(text[2+2*robo] < 0) {
      dirD = LOW;
      text[2+2*robo] = text[2+2*robo] * -1;
    }
    else {
      dirD = HIGH;
    }

    // Define a velocidade de cada roda
    analogWrite(PWM_MOTOR_ESQ, 2*text[1+2*robo]);
    analogWrite(PWM_MOTOR_DIR, 2*text[1+2*robo]);

    // Define o sentido de giro de cada roda
    digitalWrite(DIRECAO_PWM_MOTOR_ESQ_A, dirE);
    digitalWrite(DIRECAO_PWM_MOTOR_ESQ_B, !dirE);
    digitalWrite(DIRECAO_PWM_MOTOR_DIR_A, dirD);
    digitalWrite(DIRECAO_PWM_MOTOR_DIR_B, !dirD);
    
}


