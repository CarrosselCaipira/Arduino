//Código Transmissor - Importação das biblitoecas
#include <SPI.h>
#include <RF24.h>

//#define radioNum 0    numero do rádio transmissor, receptores serão 1,2...
//declarando objeto rádio
RF24 radio (2,3);  //pinos de CE = D2 e CSN = D3
const byte txAddr[6] = {"00001"}; // array de endereços, são utilizados como tubos, cada rádio pode ter até 6 endereços ** DEVE SER O MESMO NO RX PARA COMUNICAÇÃO
// Definindo dados a serem comunicados
#define txBufferSize 9  //Tamanho do vetor de comunicação, local onde os dados de transmissão serão colocados
const unsigned char caractere_inicial = 0x80; // Primeiro byte para indicar nova sequência de informações chegando da serial.
//unsigned char txBuffer[txBufferSize] = {caractere_inicial, 70, 70, 140, 210, 127, 140, 128, 128};
unsigned char txBuffer[txBufferSize];


void setup() {
  //inicialização do rádio e configurações de comunicaçao.
  Serial.begin(115200);
  radio.begin();
  radio.setRetries(15,15);
  //radio.setAutoAck(false);
  radio.setChannel(88);             //canal de comunicação, simboliza a frequência de comunicação, no caso 2400+88 Mhz.
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(txAddr);   // abertura do "tubo" (endereços) de comunicação, operação de escrita
  radio.stopListening();           //garantia que este é o transmissor
}

void loop() {
  bool pode_avancar = false;
  
    byte manda = 0x80;
      Serial.write(&manda, 1);
      /* garantindo que o byte foi enviado (espera enviar para seguir a execução) */
      //Serial.flush();
  
  //Transmissão de dados
  if (Serial.available()){
        Serial.readBytes(txBuffer, sizeof(txBuffer));
        /*Serial.println("=====================================================");
        Serial.print(sizeof(txBuffer));*/
        
          /*for(int i =0; i < txBufferSize; i++) {
            Serial.print(txBuffer[i]);
            Serial.print(" "); 
          }*/
        radio.write(&txBuffer, sizeof(txBuffer)); //& indica referencia a variável indicada, isso implica em utilizar o conteúdo da varíavel sem alterá-la
        Serial.println("ENVIAMOS SUAVE");
   }
   else {
        Serial.print("DEU RUIM IRMAOS");
        Serial.println("");
   }
   radio.write(&txBuffer, sizeof(txBuffer)); //& indica referencia a variável indicada, isso implica em utilizar o conteúdo da varíavel sem alterá-la

}
