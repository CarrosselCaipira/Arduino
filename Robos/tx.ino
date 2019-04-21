/* CODIGO ARDUINO DO RÁDIO QUE ENVIA AS INFORMAÇÕES AOS ROBÔS. */
#include "config.h"

#define RADIO_ENABLE 2 /* O pino ligado ao Chip Enable no módulo do rádio */
#define RADIO_SELECT 3 /* O pino ligado ao Chip Select no módulo do rádio */

// Portas CE e CSN do SPI
RF24 radio(RADIO_ENABLE, RADIO_SELECT);

unsigned char txBuffer[Config::BUFFER_SIZE] = {128}; /* Buffer usado para armazenar os bytes que chegaram a Serial e serem enviados através do rádio para os robôs. */

void setup() {
  //inicialização do rádio e configurações de comunicaçao.
  Serial.begin(Config::SERIAL_BIT_RATE);
  radio.begin();
  //radio.setRetries(15,15);
  //radio.setPayloadSize(6);          //Setando o tamanho dos pacotes que serão enviados, no caso 4 (limitando o tamanho do txbuffer para garantir a comunicação e evitar perca de dados
  //radio.setAutoAck(false);
  radio.setChannel(Config::CANAL);             //canal de comunicação, simboliza a frequência de comunicação, no caso 2400+88 Mhz.
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(Config::PIPE_CHAVE);   // abertura do "tubo" (endereços) de comunicação, operação de escrita
  radio.stopListening();           //garantia que este é o transmissor
  radio.printDetails();
}

void loop() {
  /* envia um byte para o pc */
  Serial.write(&Config::CARACTERE_INICIAL, 1);

  /* se prepara para receber o array do pc */
  for(;;) {
    /* aguardando até a serial ficar disponível */
    if(Serial.available()) {
      /* lendo o que vem da serial (já vem com o primeiro byte 0x80.) */
      Serial.readBytes(txBuffer, Config::BUFFER_SIZE);

      /* BEGIN DEBUG */
      //Transmissão de dados
      for(int i =0; i < Config::BUFFER_SIZE; i++) {
        Serial.print(txBuffer[i]);
        Serial.print(" ");
      }
      /* END DEBUG */

      /* checando se o byte recebido é valido */
      if(txBuffer[0] == Config::CARACTERE_INICIAL) {
        radio.write(&txBuffer, sizeof(txBuffer)); //& indica referencia a variável indicada, isso implica em utilizar o conteúdo da varíavel sem alterá-la
      }
      else {
        Serial.println("CARACTERE_INICIAL Errado.");
      }
      /* recebemos o array, parando o loop e voltando a enviar o byte para o pc solicitando o proximo array. */
      break;
    }
  }
}
