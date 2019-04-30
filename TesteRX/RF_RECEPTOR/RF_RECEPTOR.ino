#include <SPI.h>
#include <RF24.h>
//----------- DECLARAÇÕES RELACIONADAS AO RÁDIO -------------//
#define radioNum 1      // numero do rádio receptor, receptores serão 1,2...N
#define rxBufferSize 9 // tamanho do vetor que irá armazenar os dados recebidos, deve ser igual no RF_Transmissor
#define robo 0
//declarando objeto rádio
RF24 radio (2,3);  //pinos de CE = D2 e CSN = D3
const byte txAddr[6] = {"00001"}; // array de endereços, são utilizados como PIPE, cada rádio pode ter até 6 endereços ** DEVE SER O MESMO NO RX PARA COMUNICAÇÃO
byte rxBuffer[rxBufferSize];
//------------ Declarações Relacionadas aos motores -----------//
#define pwmMotorEsq 5
#define dirPWMEsqA 6
#define dirPWMEsqB 7
#define dirPWMDirA 8
#define dirPWMDirB 9
#define pwmMotorDir 10
bool dirD, dirE;
byte velEsq, velDir;
int velPWME, velPWMD;
int contador = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(pwmMotorEsq, OUTPUT);
  pinMode(dirPWMEsqA, OUTPUT);
  pinMode(dirPWMEsqB, OUTPUT);
  pinMode(dirPWMDirA, OUTPUT);
  pinMode(dirPWMDirB, OUTPUT);
  pinMode(pwmMotorDir, OUTPUT);
  
  Serial.begin(115200);
  radio.begin();
  radio.setChannel(88);             //canal deve ser o mesmo de transmissão
  radio.setPALevel(RF24_PA_LOW);      //potência do módulo em baixa, para maiores distâncias aumentar a potência
  radio.openReadingPipe(0, txAddr); //abertura do tubo de leitura
  radio.startListening();         //garantir que o rádio é o receptor
}

void loop() {
  // Rotina do receptor
    //delay(400);
  if (radio.available()) {
    
    radio.read(&rxBuffer, sizeof(rxBuffer));
    if (rxBuffer[0] == '0x80'){
        andar();
    }
    else {
      for (int i = 0; i < rxBufferSize; i++){
          Serial.print(rxBuffer[i]);
          Serial.print(" ");
        }
        Serial.println("");
    }
    /*Serial.print("Velocidades");
    Serial.println("");
    Serial.println("vel. roda esquerda ");
    Serial.print(".......");
    Serial.print(velPWME);
    Serial.println("");
    Serial.println("vel. roda direita");
    Serial.print(".......");
    Serial.print(velPWMD);
    Serial.println("");*/
  }
  else {
    Serial.print("Rádio Indisponível");
    Serial.println("");
    parar();
  }
}
void andar(){ //rotina para atuação dos motores e movimentação do robo.
  //Mapeamento do PWM na estrutura veloDirecao
  velEsq = rxBuffer[1+2*robo];
  velDir = rxBuffer[2+2*robo];
  /*Serial.println("####################################"); 
  Serial.print("velEsq: ");
  Serial.println(velEsq);
  Serial.print("velDir: ");
  Serial.println(velDir);
  Serial.println("####################################"); */
  velPWME = map(velEsq,0,255,0,255); // 127 velocidade máxima em uma direção
  velPWMD = map(velDir,0,255,0,255);
  //Verificação e definição das direções e velocidades.
  //roda esquerda
  if (rxBuffer[1+2*robo] >= 128) { // se for maior que 128 (velocidade e posição no sentido contrário)
    dirE = LOW;
  }else{                          // senão, mantém direção.
    dirE = HIGH;
  }
  // roda direita
  if (rxBuffer[2+2*robo] >= 128) {
    dirD = LOW;
  }else{
    dirD = HIGH;
  }

  velPWME = velPWME *2 + 50;
  velPWMD = velPWMD *2 + 50;
 
  analogWrite(pwmMotorEsq,velPWME);
  analogWrite(pwmMotorDir,velPWMD);
  //posições
  digitalWrite(dirPWMEsqA, dirE);
  digitalWrite(dirPWMEsqB, !dirE);
  digitalWrite(dirPWMDirA, dirD);
  digitalWrite(dirPWMDirB, !dirD);
}

void parar() {
  // Rotina caso o rádio transmissor não esteja mais disponível.
        analogWrite(pwmMotorEsq,0);
        analogWrite(pwmMotorDir,0);  
}
  
