/* CODIGO ARDUINO DO RÁDIO QUE ENVIA AS INFORMAÇÕES AOS ROBÔS. */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h> /* http://tmrh20.github.io/RF24/ */

#define RADIO_ENABLE 2 /* O pino ligado ao Chip Enable no módulo do rádio */
#define RADIO_SELECT 3 /* O pino ligado ao Chip Select no módulo do rádio */
#define SERIAL_BIT_RATE 115200 /* Pode ser definido para 115200 ou 9600. Lembre-se de que este valor deve ser o mesmo do usado pela classe do rádio, encontrada em radio.hpp.e em RX.ino */

#define TX_BUFFER_SIZE 9 /* (conferir, parecer estarmos usando 7, 2 por robo e 1 para o 0x80) tamanho do buffer utilizado para armazenar os dados enviados. Deve ser definido igual em RX.ino, TX.ino e em radio.hpp */

#define TEMPO_RETRY 15 /* intervalo de tempo entre cada tentativa de leitura. Definido em multiplos de 250us, portanto 0 = 250us e 15 = 4000us. (us: microsegundos) */
#define NUM_RETRY 15 /* número de tentativas antes de desistir da leitura */

// Portas CE e CSN do SPI
RF24 radio(RADIO_ENABLE, RADIO_SELECT);

// Identificador do rádio
const byte txChave[6] = { 'U', 'n', 'e', 's', 'p' }; /* Chave para comunicação entre RX e TX. Deve ser a mesma em ambos os códigos. */

unsigned char txBuffer[TX_BUFFER_SIZE] = {128}; /* Buffer usado para armazenar os bytes que chegaram a Serial e serem enviados através do rádio para os robôs. */

const unsigned char caractere_inicial = 0x80; /* primeiro byte da transmissao - fixo */

const int numero = 0; /* DEBUG */

void setup()
{
  Serial.begin(SERIAL_BIT_RATE);
  radio.begin();

  /* configurações de re-tentativas */
  radio.setRetries(TEMPO_RETRY, NUM_RETRY);
  /* abrindo pipe para escrita */
  radio.openWritingPipe(txChave);

  /* ativando modo de transmissao */
  radio.stopListening();

  Serial.println("Setup Completo");
}

void loop()
{
  for(;;) {
    if(Serial.available()) {
      /* lendo o que vem da serial (já vem com o primeiro byte 0x80.) */
      Serial.readBytes(txBuffer, TX_BUFFER_SIZE);

      /* fazendo o envio não bloqueante (apenas bloqueia caso o buffer esteja cheio). */
      if(!radio.writeFast(txBuffer, sizeof(txBuffer)))
        Serial.println("Falha no Envio dos Dados");
    }
  }
}
