#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Portas CE e CSN do SPI
RF24 radio(2, 3);

// Identificador do rádio
const byte rxAddr[6] = { 'U', 'n', 'e', 's', 'p' };
unsigned char rxBuf[9] = {128};
unsigned char caractere_inicial = 0;
// Debug
int numero = 0;

void setup()
{
  Serial.begin(9600);
  radio.begin();
  radio.setRetries(15, 15);
  radio.openWritingPipe(rxAddr);
  
  radio.stopListening();
}

void loop()
{
  if(Serial.available() > 0) {
    Serial.readBytes(rxBuf[1], 6);
  }
      caractere_inicial = 0x80;
      rxBuf[0] = caractere_inicial;
      // Envia os dados
      radio.write(&rxBuf, sizeof(rxBuf));
      Serial.println(numero);
      delay(250);
    caractere_inicial = 0; 
}
