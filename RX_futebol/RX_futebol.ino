#define NUM_ROBO 0
//#define NUM_ROBO 1
//#define NUM_ROBO 2

//#define DEBUG
//#define RADIOBIDIR

#include <SPI.h>
#include "API.h"
#include "nRF24L01.h"

//***************************************************
#define TX_ADR_WIDTH    5   // 5 bytes TX(RX) address width
#define TX_PLOAD_WIDTH  9   // 9 bytes TX payload
#define PWM_MOTOR_ESQ 5
#define DIRECAO_PWM_MOTOR_ESQ_A 6
#define DIRECAO_PWM_MOTOR_ESQ_B 7
#define DIRECAO_PWM_MOTOR_DIR_A 8
#define DIRECAO_PWM_MOTOR_DIR_B 9
#define PWM_MOTOR_DIR 10

#define ODOMETRIA_ESQ A0
#define ODOMETRIA_DIR A1
#define TESTE_ODOMETRIA_ESQ A2
#define TESTE_ODOMETRIA_DIR A3
byte contOdoEsq = 0;//contador de pulsos, zerado a cada inicio de contagem (a cada novo comando)
byte contOdoDir = 0;
char ultContagemOdoEsq = 0;//ultima contagem desde a ultima medida
char ultContagemOdoDir = 0;

byte TX_ADDRESS[TX_ADR_WIDTH] = { 'D', 'C', 'o', 'F', 'C' }; // Define a static TX address

byte rx_buf[TX_PLOAD_WIDTH];
byte tx_buf[TX_PLOAD_WIDTH] = {0xff};
//***************************************************
    byte	kp[8]={1,  8,  7,  6,  5,  4,  3,  2},
		ki[8]={1,  2,  2,  2,  2,  2,  2,  2},
		kd[8]={1,  2,  0,  0,  0,  0,  0,  0},
		kr[8]={1, 16, 16, 16, 16, 16, 16, 16},	// relacao entre as rodas
		kv[8]={0, 24, 27, 32, 39, 48, 59, 72};	// integral inicial
    byte	vOdo[]={0, 3,  6,  9, 12, 16, 19, 22}; // a indice é a velocidade e os valores relacionados aos ìndices são os pulsos do odometro.
//***************************************************
void setup() {
  Serial.begin(115200);
  pinMode(CE, OUTPUT);
  pinMode(CSN, OUTPUT);
  pinMode(IRQ, INPUT);
  SPI.begin();
  delay(50);
  init_io();                        // Initialize IO port
  byte sstatus = SPI_Read(STATUS);
  Serial.println("* RX_Mode Start *");
  Serial.print("status = ");
  Serial.println(sstatus, HEX);     // There is read the mode’s status register, the default value should be ‘E’
  RX_Mode();                        // set RX mode

  pinMode(PWM_MOTOR_ESQ, OUTPUT);
  pinMode(DIRECAO_PWM_MOTOR_ESQ_A, OUTPUT);
  pinMode(DIRECAO_PWM_MOTOR_ESQ_B, OUTPUT);
  pinMode(DIRECAO_PWM_MOTOR_DIR_A, OUTPUT);
  pinMode(DIRECAO_PWM_MOTOR_DIR_B, OUTPUT);
  pinMode(PWM_MOTOR_DIR, OUTPUT);
  pinMode(ODOMETRIA_ESQ, INPUT);
  pinMode(ODOMETRIA_DIR, INPUT);
  pinMode(TESTE_ODOMETRIA_ESQ, OUTPUT);
  pinMode(TESTE_ODOMETRIA_DIR, OUTPUT);
}

boolean dirEsq = false, dirDir = false;
byte velBaseEsq, vBaseDir;
byte vBaseEsq, velBaseDir;
int erroAntEsq , erroAntDir;
int erroEsq = 0, erroDir = 0;
int difRodas, difRodasEsq, difRodasDir;
int vMax, velEsq, velDir;

#ifdef DEBUG
void mostraSerial(char * s, int i) {
  Serial.print(s);
  if (i>=0)
  Serial.print(" ");
  if (i<100)
  Serial.print(" ");
  if (abs(i)<10)
  Serial.print(" ");
  Serial.print(i);
}
#endif

void loop() {
  for (;;) {
    byte status = SPI_Read(STATUS);                         // read register STATUS's value    
    if (dadosComunicacao()) {                                         // se verdadeiro, chegou dados e estao em rx_buf
      if (rx_buf[0] == 0x80) {
        /* As palavras que chegam do rádio são de 8 bits. O mais significativo indica a direção da roda, enquanto os
        demais indicam a velocidade.
           Abaixo ocorre um deslocamento de 4 bits para a esquerda no momento de salvar a velocidade. Isso ocorre pois
        trabalhamos com apenas 7 velocidades, não 127.
           As palavras são armazenadas no vetor rx_buf onde a posição 0 é um teste e as posições seguintes são as
        velocidades correspondentes à cada roda. Ex.: pos[1] indica a velocidade (e direcao) da roda esquerda
        enviada ao robo 0 e a pos[1] indica as mesmas coisas mas para a roda direita o mesmo robo. */
        velBaseEsq = rx_buf[NUM_ROBO*2+1] >> 4; // recebe a velocidade para a roda esquerda conforme a regra acima.
        velBaseDir = rx_buf[NUM_ROBO*2+2] >> 4; // recebe a velocidade para a roda direita conforme a regra acima.
        dirEsq = velBaseEsq & 0x08; // primeiro bit que chega do rádio indica a direção da roda esquerda, por isso a mascara '8'(1000).
        dirDir = velBaseDir & 0x08; // primeiro bit que chega do rádio indica a direção da roda direita, por isso a mascara '8'(1000).
        vBaseEsq = velBaseEsq & 0x07; // os ultimos 3 bits indicam a velocidade da roda esquerda, por isso a mascara '7'(0111).
        vBaseDir = velBaseDir & 0x07; // os ultimos 3 bits indicam a velocidade da roda direita, por isso a mascara '7'(0111).
        // inicio dos ajustes de erro (proporcional apenas).
        // recorda do erro anterior.
        erroAntEsq = erroEsq;
        erroAntDir = erroDir;
        // calculo do novo erro com relação a cada roda.
        // utilizados valores da tabela vOdo (declarada acima) que contem valores (empíricos) da odometria resultante de cada velocidade.
        // determina o erro da roda, fazendo a diferença do que deveria ser (valor empirico de odometria) com o ultimo valor da odometria multipica por 4.
        erroEsq = ((int)vOdo[vBaseEsq]-ultContagemOdoEsq)<<2; //deslocamento para casas decimais
        erroDir = ((int)vOdo[vBaseDir]-ultContagemOdoDir)<<2; //deslocamento para casas decimais
        
        #ifdef DEBUG
        mostraSerial("a ",erroEsq);
        #endif
        
        // Envio dos dados via rádio para o computador. Sem uso no momento, quando em uso, descomentar #RADIOBIDIR.
        #ifdef RADIOBIDIR
        tx_buf[5] = erroEsq;
        tx_buf[6] = erroDir;
        tx_buf[7] = velBaseEsq;
        tx_buf[8] = velBaseDir;
        #endif
        
		/* Calcula a diferença de velocidade entre as rodas para realizar a compensação. */
        difRodas=((int)vBaseEsq*ultContagemOdoDir-(int)vBaseDir*ultContagemOdoEsq)<<4; // deslocamento para casas decimais, logo o numero será multiplicado por 16.
        
        difRodasDir = -difRodas/kr[vBaseDir];
        difRodasEsq =  difRodas/kr[vBaseEsq];
        
        erroEsq+=difRodasEsq;
        erroDir+=difRodasDir;
        
        //intEsq=(intEsq+erroEsq)/2;
        //intDir=(intDir+erroDir)/2;

        velEsq=((int)kp[vBaseEsq]*erroEsq)>>1//deslocamento para casas decimais
        //+ ki[vBaseEsq]*intEsq
        //+ kd[vBaseEsq]*(erroEsq-erroAntEsq)
        ;

        velDir=((int)kp[vBaseDir]*erroDir)>>1//deslocamento para casas decimais
        //+ ki[vBaseDir]*intDir
        //+ kd[vBaseDir]*(erroDir-erroAntDir)
        ;

        #ifdef DEBUG
        mostraSerial("b ", velEsq);
        #endif

        velEsq+=kv[vBaseEsq];
        velDir+=kv[vBaseDir];
        
        if (velEsq<0) { // a velocidade pode ser negativa dependendo do valor do erro.
          velEsq = -velEsq;
          dirEsq = !dirEsq; // o robo tenta inveter o sentido da roda para parar mais rapido.
          velEsq = 0; // a velocidade é zerada pois o odometro não identifica o lado que a roda esta girando.
        }
        if (velDir<0) { // a velocidade pode ser negativa dependendo do valor do erro.
          velDir = -velDir;
          dirDir = !dirDir; // o robo tenta inveter o sentido da roda para parar mais rapido.
          velDir = 0; // a velocidade é zerada pois o odometro não identifica o lado que a roda esta girando.
        }
        
        #ifdef DEBUG
        mostraSerial("c ", velEsq);
        mostraSerial("d ", vBaseEsq);
        #endif
       
        /*Serial.print("E:");
        Serial.print(tx_buf[1]);
        Serial.print("D:");
        Serial.print(tx_buf[2]);*/
        
        vMax = max(velEsq, velDir);
        if (vMax>250) { // a velocidade nao pode ser mais de 250 pois nao podemos usar mais que toda a tensão dispoivel no motor.
          velEsq = ((250>>1)*(unsigned)velEsq/vMax)<<1; // deslocamentos (para frente e para tras) feitos para arredondamento dos valores.
          velDir = ((250>>1)*(unsigned)velDir/vMax)<<1; // deslocamentos (para frente e para tras) feitos para arredondamento dos valores.
        }

        //controle de velocidade e sentido da roda esquerda.
        if (vBaseEsq!=0) {
          analogWrite(PWM_MOTOR_ESQ, velEsq);
          if (dirEsq) { // andando para tras.
            digitalWrite(DIRECAO_PWM_MOTOR_ESQ_A, HIGH);
            digitalWrite(DIRECAO_PWM_MOTOR_ESQ_B, LOW);
          } else { // andando para frete
            digitalWrite(DIRECAO_PWM_MOTOR_ESQ_A, LOW);
            digitalWrite(DIRECAO_PWM_MOTOR_ESQ_B, HIGH);
          }
        } else { // o robô deve parar.
          analogWrite(PWM_MOTOR_ESQ, 255);//Parada Rapida L298
          digitalWrite(DIRECAO_PWM_MOTOR_ESQ_A, LOW);
          digitalWrite(DIRECAO_PWM_MOTOR_ESQ_B, LOW);
        }

        //controle de velocidade e sentido da roda direita.
        if (vBaseDir!=0) {
          analogWrite(PWM_MOTOR_DIR, velDir);
          if (dirDir) { // andando para tras.
            digitalWrite(DIRECAO_PWM_MOTOR_DIR_A, HIGH);
            digitalWrite(DIRECAO_PWM_MOTOR_DIR_B, LOW);
          } else { //andando para frente.
            digitalWrite(DIRECAO_PWM_MOTOR_DIR_A, LOW);
            digitalWrite(DIRECAO_PWM_MOTOR_DIR_B, HIGH);
          }
        } else { // o robô deve parar.
          analogWrite(PWM_MOTOR_DIR, 255);
          digitalWrite(DIRECAO_PWM_MOTOR_DIR_A, LOW);
          digitalWrite(DIRECAO_PWM_MOTOR_DIR_B, LOW);
        }
        
        /*Serial.print(" E:");
        Serial.print(velEsq);
        Serial.print("D:");
        Serial.print(velDir);*/
        // Envio dos dados via rádio para o computador. Sem uso no momento, quando em uso, descomentar #RADIOBIDIR.
        #ifdef RADIOBIDIR
        tx_buf[3] = velEsq;
        tx_buf[4] = velDir;
        #endif
        
        /*
        analogWrite(PWM_MOTOR_ESQ, (rx_buf[NUM_ROBO + 1] & 0x7f) << 1);
        if (rx_buf[NUM_ROBO + 1] & 0x80) {
          digitalWrite(DIRECAO_PWM_MOTOR_ESQ_A, HIGH);
          digitalWrite(DIRECAO_PWM_MOTOR_ESQ_B, LOW);
        } else {
          digitalWrite(DIRECAO_PWM_MOTOR_ESQ_A, LOW);
          digitalWrite(DIRECAO_PWM_MOTOR_ESQ_B, HIGH);
        }

        analogWrite(PWM_MOTOR_DIR, (rx_buf[NUM_ROBO + 2] & 0x7f) << 1);
        if (rx_buf[NUM_ROBO + 2] & 0x80) {
          digitalWrite(DIRECAO_PWM_MOTOR_DIR_A, HIGH);
          digitalWrite(DIRECAO_PWM_MOTOR_DIR_B, LOW);
        } else {
          digitalWrite(DIRECAO_PWM_MOTOR_DIR_A, LOW);
          digitalWrite(DIRECAO_PWM_MOTOR_DIR_B, HIGH);
        }
        */
      }
      /*for (int i = 0; i < 3; i++) {
        Serial.print(" ");
        Serial.print(rx_buf[i], HEX);                              // print rx_buf
      }
      Serial.println(" ");*/
    }
    //SPI_RW_Reg(WRITE_REG + STATUS, status);                             // clear RX_DR or TX_DS or MAX_RT interrupt flag

    verificaOdoEsq();
    verificaOdoDir();
  }
}

boolean dadosComunicacao() {
  byte sstatus = SPI_Read(STATUS);                         // read register STATUS's value
  if (sstatus & RX_DR) {     
    tx_buf[1] = ultContagemOdoEsq = contOdoEsq;
    contOdoEsq = 0;
    tx_buf[2] = ultContagemOdoDir = contOdoDir;
    contOdoDir = 0;
    //SPI_RW_Reg(FLUSH_TX, 0);
    
    #ifdef DEBUG
    SPI_Write_Buf(W_ACK_PAYLOAD, tx_buf, TX_PLOAD_WIDTH);             //W_ACK_PAYLOAD
    for (int i = 0; i < TX_PLOAD_WIDTH; i++) {
        mostraSerial("", tx_buf[i]);                              // print rx_buf
    }
    Serial.print("\n");
    #endif
    SPI_Read_Buf(RD_RX_PLOAD, rx_buf, TX_PLOAD_WIDTH);             // read playload to rx_buf
    SPI_RW_Reg(FLUSH_RX, 0);
  }
  SPI_RW_Reg(WRITE_REG + STATUS, sstatus);                     // clear RX_DR or TX_DS or MAX_RT interrupt flag
  return sstatus & RX_DR;
}

const int VARIACAO_IGNORAVEL = 100;

void verificaOdoEsq() {
  static boolean subindo = true;
  static unsigned long tempoInicio = 0;
  static unsigned long tempoMudancaFutura = 0;
  static int valMin = 0;
  static int valMax = 0;

  int valAtual = analogRead(ODOMETRIA_ESQ);

  unsigned long tempoAtual = millis();
  if (tempoMudancaFutura != 0)
    if (tempoAtual >= tempoMudancaFutura) {
      tempoMudancaFutura = 0;
      contOdoEsq++;
      digitalWrite(TESTE_ODOMETRIA_ESQ, LOW);  // Para teste com o ociloscópio. Gera uma onda quadrada.
    }
  if (subindo) { // Estado da função da forma de onda.
    if (valAtual > valMax)
      valMax = valAtual;
    else if (valMax - valAtual > VARIACAO_IGNORAVEL) {
      subindo = false;
      valMin = valMax;
      goto contagem;
    }
  } else { // Descendo. Estado da função da forma de onda.
    if (valAtual < valMin)
      valMin = valAtual;
    else if (valAtual - valMin > VARIACAO_IGNORAVEL) {
      subindo = true;
      valMax = valMin;
      goto contagem;
    }
  }
  return;
  contagem: { //contagem que ocorre apenas quando o tempo medio do meio ciclo anterior é maior que o meio ciclo atual
    contOdoEsq++;
    digitalWrite(TESTE_ODOMETRIA_ESQ, HIGH);  // Para teste com o ociloscópio. Gera uma onda quadrada.
    if (tempoMudancaFutura != 0) { // se a contagem pelo tempo ainda nao foi feita e o meio ciclo ja acabou (meio ciclo muito menor que anterior), conta
      contOdoEsq++;
      digitalWrite(TESTE_ODOMETRIA_ESQ, LOW);  // Para teste com o ociloscópio. Gera uma onda quadrada.
    }
    tempoMudancaFutura = tempoAtual + (tempoAtual - tempoInicio) / 2;
    tempoInicio = tempoAtual;
  }
  return;
}

void verificaOdoDir() {
  static boolean subindo = true;
  static unsigned long tempoInicio = 0;
  static unsigned long tempoMudancaFutura = 0;
  static int valMin = 0;
  static int valMax = 0;

  int valAtual = analogRead(ODOMETRIA_DIR);

  unsigned long tempoAtual = millis();
  if (tempoMudancaFutura != 0)
    if (tempoAtual >= tempoMudancaFutura) {
      tempoMudancaFutura = 0;
      contOdoDir++;
      digitalWrite(TESTE_ODOMETRIA_DIR, LOW);  // Para teste com o ociloscópio. Gera uma onda quadrada.
    }
  if (subindo) { // Estado da função da forma de onda.
    if (valAtual > valMax)
      valMax = valAtual;
    else if (valMax - valAtual > VARIACAO_IGNORAVEL) {
      subindo = false;
      valMin = valMax;
      goto contagem;
    }
  } else { // Descendo. Estado da função da forma de onda.
    if (valAtual < valMin)
      valMin = valAtual;
    else if (valAtual - valMin > VARIACAO_IGNORAVEL) {
      subindo = true;
      valMax = valMin;
      goto contagem;
    }
  }
  return;
  contagem: { //contagem que ocorre apenas quando o tempo medio do meio ciclo anterior é maior que o meio ciclo atual
    contOdoDir++;
    digitalWrite(TESTE_ODOMETRIA_DIR, HIGH);  // Para teste com o ociloscópio. Gera uma onda quadrada.
    if (tempoMudancaFutura != 0) { // se a contagem pelo tempo ainda nao foi feita e o meio ciclo ja acabou (meio ciclo muito menor que anterior), conta
      contOdoDir++;
      digitalWrite(TESTE_ODOMETRIA_DIR, LOW);  // Para teste com o ociloscópio. Gera uma onda quadrada.
    }
    tempoMudancaFutura = tempoAtual + (tempoAtual - tempoInicio) / 2;
    tempoInicio = tempoAtual;
  }
  return;
}

//**** a partir daqui vem o cdigo do SPI + tranceptor *****
/*********************************************************************
 **  Device:  nRF24L01+                                              **
 **  File:   EF_nRF24L01_RX.c                                        **
 **                                                                  **
 **                                                                  **
 **  Copyright (C) 2011 ElecFraks.                                   **
 **  This example code is in the public domain.                      **
 **                                                                  **
 **  Description:                                                    **
 **  This file is a sample code for your reference.                  **
 **  It's the v1.0 nRF24L01+ Hardware SPI by arduino                 **
 **  Created by ElecFreaks. Robi.W,11 June 2011                      **
 **                                                                  **
 **  http://www.elecfreaks.com                                       **
 **                                                                  **
 **   SPI-compatible                                                 **
 **   CS - to digital pin 3               - preto                    **
 **   CSN - to digital pin 4  (SS pin)    - marrom                   **
 **   MOSI - to digital pin 11 (MOSI pin) - laranja                  **
 **   MISO - to digital pin 12 (MISO pin) - amarelo                  **
 **   CLK - to digital pin 13 (SCK pin)   - vermelho                 **
 *********************************************************************/
//**************************************************
// Function: init_io();
// Description:
// flash led one time,chip enable(ready to TX or RX Mode),
// Spi disable,Spi clock line init high
//**************************************************
void init_io(void) {
  digitalWrite(IRQ, 0);
  digitalWrite(CE, 0);      // chip enable
  digitalWrite(CSN, 1);                 // Spi disable
}

/************************************************************************
 **   * Function: SPI_RW();
 *
 * Description:
 * Writes one unsigned char to nRF24L01, and return the unsigned char read
 * from nRF24L01 during write, according to SPI protocol
 ************************************************************************/
unsigned char SPI_RW(unsigned char Byte) {
  return SPI.transfer(Byte);
}

/**************************************************/

/**************************************************
 * Function: SPI_RW_Reg();
 *
 * Description:
 * Writes value 'value' to register 'reg'
 /**************************************************/
unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value) {
  unsigned char status;

  digitalWrite(CSN, 0);                   // CSN low, init SPI transaction
  SPI_RW(reg);                            // select register
  SPI_RW(value);                          // ..and write value to it..
  digitalWrite(CSN, 1);                   // CSN high again

  return (status);                   // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Read();
 *
 * Description:
 * Read one unsigned char from nRF24L01 register, 'reg'
 /**************************************************/
unsigned char SPI_Read(unsigned char reg) {
  unsigned char reg_val;

  digitalWrite(CSN, 0);                // CSN low, initialize SPI communication...
  SPI_RW(reg);                         // Select register to read from..
  reg_val = SPI_RW(0);                 // ..then read register value
  digitalWrite(CSN, 1);                // CSN high, terminate SPI communication

  return (reg_val);                     // return register value
}
/**************************************************/

/**************************************************
 * Function: SPI_Read_Buf();
 *
 * Description:
 * Reads 'unsigned chars' #of unsigned chars from register 'reg'
 * Typically used to read RX payload, Rx/Tx address
 /**************************************************/
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes) {
  unsigned char sstatus, i;

  digitalWrite(CSN, 0);                   // Set CSN low, init SPI tranaction
  sstatus = SPI_RW(reg);            // Select register to write to and read status unsigned char

  for (i = 0; i < bytes; i++) {
    pBuf[i] = SPI_RW(0);    // Perform SPI_RW to read unsigned char from nRF24L01
  }

  digitalWrite(CSN, 1);                   // Set CSN high again

  return (sstatus);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Write_Buf();
 *
 * Description:
 * Writes contents of buffer '*pBuf' to nRF24L01
 * Typically used to write TX payload, Rx/Tx address
 /**************************************************/
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes) {
  unsigned char sstatus, i;

  digitalWrite(CSN, 0);                   // Set CSN low, init SPI tranaction
  sstatus = SPI_RW(reg);             // Select register to write to and read status unsigned char
  for (i = 0; i < bytes; i++) {          // then write all unsigned char in buffer(*pBuf)
    SPI_RW(*pBuf++);
  }
  digitalWrite(CSN, 1);                   // Set CSN high again
  return (sstatus);                  // return nRF24L01 status unsigned char
}
/**************************************************/

void RX_Mode(void) {
  digitalWrite(CE, 0);

  SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // RX_Addr0 same as TX_Adr for Auto.Ack

  SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
  SPI_RW_Reg(WRITE_REG + RF_CH, 'R');        // Select RF channel 40
  SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width
  SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   // TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:RX. RX_DR enabled..

  #ifdef DEBUG
  SPI_RW_Reg(WRITE_REG + FEATURE, 6);       //Enables Dynamic Payload Length e Payload with ACK
  SPI_RW_Reg(WRITE_REG + DYNPD, 1);
  #endif

  SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);

  digitalWrite(CE, 1);                             // Set CE pin high to enable RX device
  //  This device is now ready to receive one packet of 16 unsigned chars payload from a TX device sending to address
  //  '3443101001', with auto acknowledgment, retransmit count of 10, RF channel 40 and datarate = 2Mbps.
}

