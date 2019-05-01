/* CÓDIGO ARDUINO DOS ROBÔS QUE RECEBEM AS INFORMAÇÕES DO RÁDIO. */
#include "config.h"

/* ------------ Declarações Relacionadas ao Robo que terá este código gravodo ----------- */
/* Define qual robo é esse (valores possíveis: 0, 1 ou 2) */
const byte NUM_ROBO = 0;

/* Somamos 1 no inicio pois o primeiro elemento do buffer é fixo, nomalmente 0x80. */
const int INDEX_RODA_ESQ = 1 + 2 * NUM_ROBO; /* index da roda esquerda no array rxBuffer. */
const int INDEX_RODA_DIR = 2 + 2 * NUM_ROBO; /* index da roda direita no array rxBuffer. */

/* ------------ Declarações Relacionadas aos motores ----------- */
#define PWM_MOTOR_ESQ 5
#define DIRECAO_PWM_MOTOR_ESQ_A 6
#define DIRECAO_PWM_MOTOR_ESQ_B 7
#define DIRECAO_PWM_MOTOR_DIR_A 8
#define DIRECAO_PWM_MOTOR_DIR_B 9
#define PWM_MOTOR_DIR 10

/* ------------ Declarações Relacionadas aos Pinos do Rádio ----------- */
#define RADIO_ENABLE 2 /* O pino ligado ao Chip Enable no módulo do rádio */
#define RADIO_SELECT 3 /* O pino ligado ao Chip Select no módulo do rádio */

RF24 radio(RADIO_ENABLE, RADIO_SELECT);
byte rxBuffer[Config::BUFFER_SIZE]; /* Buffer usado para armazenar os bytes recebidos através do rádio. */

void setup() {
  // put your setup code here, to run once:
  pinMode(PWM_MOTOR_ESQ, OUTPUT);
  pinMode(DIRECAO_PWM_MOTOR_ESQ_A, OUTPUT);
  pinMode(DIRECAO_PWM_MOTOR_ESQ_B, OUTPUT);
  pinMode(DIRECAO_PWM_MOTOR_DIR_A, OUTPUT);
  pinMode(DIRECAO_PWM_MOTOR_DIR_B, OUTPUT);
  pinMode(PWM_MOTOR_DIR, OUTPUT);

  Serial.begin(Config::SERIAL_BIT_RATE);
  radio.begin();
  radio.setChannel(Config::CANAL);             //canal deve ser o mesmo de transmissão
  radio.setPALevel(RF24_PA_LOW);      //potência do módulo em baixa, para maiores distâncias aumentar a potência
  radio.openReadingPipe(Config::IND_PIPE_LEITURA, Config::PIPE_CHAVE); //abertura do tubo de leitura
  radio.startListening();         //garantir que o rádio é o receptor
}

void loop() {
  // Rotina do receptor
  if (radio.available()) {
    /* lendo o buffer do rádio */
    radio.read(&rxBuffer, sizeof(rxBuffer));

    /* BEGIN DEBUG
    for (int i = 0; i < Config::BUFFER_SIZE; i++){
      Serial.print(rxBuffer[i]);
      Serial.println("");
    }
    END DEBUG */

    andar(rxBuffer[INDEX_RODA_ESQ], rxBuffer[INDEX_RODA_DIR]);
  }
  else {
    Serial.println("Rádio Indisponível");
  }
}

//rotina para atuação dos motores e movimentação do robo.
void andar(byte velEsq, byte velDir) {
  bool dirD, dirE;
  int velPWMEsq, velPWMDir;
  
  // verifica roda esquerda
  /* (expression) ? (true-value) : (false-value). Logo, se velEsq < 100 então dirE é LOW, do contrário (velEsq >= 100), dirE é HIGH */
  dirE = velEsq < 100 ? LOW : HIGH;
  // verifica roda direita
  dirD = velDir < 100 ? LOW : HIGH;

  //Mapeamento do PWM na estrutura veloDirecao
  velPWMEsq = map(velEsq,0,127,0,255);
  velPWMDir = map(velDir,0,127,0,255);
  //velPWMEsq = velPWMEsq*2;
  //velPWMDir = velPWMDir*2;

  //definindo sentidos de movimento (frente, trás, giro esquerda e giro direita)
  // Define a velocidade de cada roda
  analogWrite(PWM_MOTOR_ESQ,velPWMEsq);
  analogWrite(PWM_MOTOR_DIR,velPWMDir);

  // Define o sentido de giro de cada roda
  digitalWrite(DIRECAO_PWM_MOTOR_ESQ_A, dirE);
  digitalWrite(DIRECAO_PWM_MOTOR_ESQ_B, !dirE);
  digitalWrite(DIRECAO_PWM_MOTOR_DIR_A, dirD);
  digitalWrite(DIRECAO_PWM_MOTOR_DIR_B, !dirD);
}
