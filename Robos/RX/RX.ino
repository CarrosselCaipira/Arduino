/* CÓDIGO ARDUINO DOS ROBÔS QUE RECEBEM AS INFORMAÇÕES DO RÁDIO. */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h> /* http://tmrh20.github.io/RF24/ */

#define ODOMETRIA_ESQ A0
#define ODOMETRIA_DIR A1
#define TESTE_ODOMETRIA_ESQ A2
#define TESTE_ODOMETRIA_DIR A3

#define PWM_MOTOR_ESQ 5
#define DIRECAO_PWM_MOTOR_ESQ_A 6
#define DIRECAO_PWM_MOTOR_ESQ_B 7
#define DIRECAO_PWM_MOTOR_DIR_A 8
#define DIRECAO_PWM_MOTOR_DIR_B 9
#define PWM_MOTOR_DIR 10

#define RADIO_ENABLE 2 /* O pino ligado ao Chip Enable no módulo do rádio */
#define RADIO_SELECT 3 /* O pino ligado ao Chip Select no módulo do rádio */
#define SERIAL_BIT_RATE 9600 /* Pode também ser definido para 115200. Lembre-se de que este valor deve ser o mesmo do usado pela classe do rádio, encontrada em radio.hpp.e em TX.ino */

#define PIPE_LEITURA 0 /* indica qual pipe será aberta para leitura das informações vindas do rádio. Valores possíveis: [0,5]. 0 e 1 podem usar endereços de 5 bytes, os demais apenas 1. Talvez seja interessante trocar para 1 caso consideremos a comunicação bidirecional pois as escritas ocorrem  nesse pipe. */

#define RX_BUFFER_SIZE 9 /* tamanho do buffer utilizado para armazenar os dados recebidos. Deve ser definido igual em RX.ino, TX.ino e em radio.hpp */

/* contadores de pulsos de odometria, é zerado a cada novo comando vindo do rádio */
/* ATUAIS */
byte contOdoEsq = 0;
byte contOdoDir = 0;
/* ANTERIORES */
char ultContagemOdoEsq = 0;
char ultContagemOdoDir = 0;

/* Velocidades mínimas para os motores vencerem a inércia. CORRETO? */
/* const? */ int valMinEsq = 1000;
/* const? */ int valMinDir = 1000;

/* const? */ int valMaxEsq = 0; /* ??? */
/* const? */ int valMaxDir = 0; /* ??? */

unsigned long tempoAnterior = 0; /* ??? */

int numero = 0; /* ??? */
int erro = 0; /* ??? */

/* Define qual robo é esse (valores possíveis: 0, 1 ou 2) */
const int robo = 0;

const int indexRodaEsq = 1 + 2 * robo; /* index da roda esquerda no rxBuffer. */
const int indexRodaDir = 2 + 2 * robo; /* index da roda direita no rxBuffer. */

RF24 radio(RADIO_ENABLE, RADIO_SELECT);

byte rxBuffer[RX_BUFFER_SIZE]; /* Buffer usado para armazenar os bytes recebidos através do rádio. */

boolean dirE = false; /* necessarios mais detalhes. Indica o sentido de rotação? */
boolean dirD = false; /* necessarios mais detalhes. Indica o sentido de rotação? */

// Identificador do rádio
const byte rxChave[6] = { 'U', 'n', 'e', 's', 'p' }; /* Chave para comunicação entre RX e TX. Deve ser a mesma em ambos os códigos. */

void setup() {
  pinMode(PWM_MOTOR_ESQ, OUTPUT);
  pinMode(DIRECAO_PWM_MOTOR_ESQ_A, OUTPUT);
  pinMode(DIRECAO_PWM_MOTOR_ESQ_B, OUTPUT);
  pinMode(DIRECAO_PWM_MOTOR_DIR_A, OUTPUT);
  pinMode(DIRECAO_PWM_MOTOR_DIR_B, OUTPUT);
  pinMode(PWM_MOTOR_DIR, OUTPUT);
  pinMode(ODOMETRIA_ESQ, INPUT);
  pinMode(ODOMETRIA_DIR, INPUT);

  while (!Serial);
  Serial.begin(SERIAL_BIT_RATE);

  radio.begin();

  /* Abrindo o pipe para leitura utilizando.  */
  radio.openReadingPipe(PIPE_LEITURA, rxAddr);

  /* Habilitando o radio para a leitura propriamente dita. */
  radio.startListening();
}

void loop() {
  /* se o radio esteja disponível... recebe as informações de velocidade e anda. */
  if(com()) {
    anda();
  }

  //verificaOdoEsq();
  //verificaOdoDir();
}

/* Recebe comandos novos */
bool com() {
  bool got = radio.available();

  if (got) {
    radio.read(&rxBuffer, sizeof(rxBuffer));
    if(rxBuffer[0] == 0x80){
          Serial.println("Recebemos algo");
    }
  }
}

// Movimenta os motores
void anda() {

  byte velRodaEsq, velRodaDir; /* velocidades que chegaram do radio */

  dirE = HIGH;
  dirD = HIGH;

// Atribui as velocidades

  /*
    // Trabalha os valores deslocados (0 - 256)
    // Verifica a direção da roda esquerda
    if(rxBuffer[1+2*robo] >= 128) {
      dirE = LOW;
      // Tira o primeiro bit
      rxBuffer[1+2*robo] = rxBuffer[1+2*robo] - 128;
    }
    else {
      dirE = HIGH;
    }

    // Verifica a direção da roda direita
    if(rxBuffer[2+2*robo] >= 128) {
      dirD = LOW;
      rxBuffer[2+2*robo] = rxBuffer[2+2*robo] - 128;
    }
    else {
      rxBuffer[2+2*robo] = rxBuffer[2+2*robo] - 128;
    }
   */

   // Verifica a direção da roda esquerda
    if(rxBuffer[indexRodaEsq] < 0) {
      dirE = LOW;
      // Tira o primeiro bit
      velRodaEsq = rxBuffer[indexRodaEsq] * -1;
    }
    else {
      dirE = HIGH;
    }

    // Verifica a direção da roda direita
    if(rxBuffer[indexRodaDir] < 0) {
      dirD = LOW;
      velRodaDir = rxBuffer[indexRodaDir] * -1;
    }
    else {
      dirD = HIGH;
    }

    // Define a velocidade de cada roda
    analogWrite(PWM_MOTOR_ESQ, 2 * velRodaEsq);
    analogWrite(PWM_MOTOR_DIR, 2 * velRodaDir);

    // Define o sentido de giro de cada roda
    digitalWrite(DIRECAO_PWM_MOTOR_ESQ_A, dirE);
    digitalWrite(DIRECAO_PWM_MOTOR_ESQ_B, !dirE);
    digitalWrite(DIRECAO_PWM_MOTOR_DIR_A, dirD);
    digitalWrite(DIRECAO_PWM_MOTOR_DIR_B, !dirD);

}
