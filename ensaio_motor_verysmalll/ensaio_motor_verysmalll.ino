/*Programa para ensaio dos motores do rob de futebol*/

#define ODOMETRIA_ESQ A0
#define ODOMETRIA_DIR A1
#define TESTE_ODOMETRIA_ESQ A2
#define TESTE_ODOMETRIA_DIR A3

byte contOdoEsq = 0;//contador de pulsos, zerado a cada inicio de contagem (a cada novo comando)
byte contOdoDir = 0;
const int VARIACAO_IGNORAVEL = 100;

//Definindo os pinos dos motores
int en_A = 10;           //PWM do motor Direito
int motor_A_frente = 8;  
int motor_A_traz = 9;    

int en_B = 5;           //PWM do motor Esquerdo
int motor_B_frente = 6;  
int motor_B_traz = 7;    



void setup (){
  
  //Configurando os enables para o pwm e os motores A e B.
  //Ambos devem ser de saida -> OUTPUT
  pinMode (en_A, OUTPUT);
  pinMode (motor_A_frente, OUTPUT);
  pinMode (motor_A_traz, OUTPUT);
  pinMode (en_B, OUTPUT);
  pinMode (motor_B_frente, OUTPUT);
  pinMode (motor_B_traz, OUTPUT);
  
  //Configurando os sensores de IR, um envia o sinal e o outro recebe
  pinMode(ODOMETRIA_ESQ, INPUT);
  pinMode(ODOMETRIA_DIR, INPUT);
  pinMode(TESTE_ODOMETRIA_ESQ, OUTPUT);
  pinMode(TESTE_ODOMETRIA_DIR, OUTPUT);
}


void loop() {
  analogWrite(en_B, 127); //definindo a velocidade do motor, vai de 0 a 255
  digitalWrite (motor_B_traz, HIGH); //ligando o motor 1 em um sentido apenas
  analogWrite(en_A, 127); //definindo a velocidade do motor, vai de 0 a 255
  digitalWrite (motor_A_traz, HIGH); //ligando o motor 1 em um sentido apenas
  unsigned long dadosMotorEsq[2];
  unsigned long dadosMotorDir[2];            
  for(;;){
    verificaOdoEsq(dadosMotorEsq);
    Serial.print("ME | Contador: ");
    Serial.println(dadosMotorEsq[0]);
    Serial.print("ME | Tempo passado: ");
    Serial.println(dadosMotorEsq[1]);
    verificaOdoDir(dadosMotorDir);
    Serial.print("MD | Contador: ");
    Serial.println(dadosMotorDir[0]);
    Serial.print("MD | Tempo passado: ");
    Serial.println(dadosMotorDir[1]);
  }
}

void verificaOdoEsq(unsigned long dados[2]) {
  static boolean subindo = true;
  static unsigned long tempoInicio = 0;
  static unsigned long tempoMudancaFutura = 0;
  static int valMin = 0;
  static int valMax = 0;
  unsigned long tempoTotal;
  int valAtual = analogRead(ODOMETRIA_ESQ);

  unsigned long tempoAtual = millis();
  if (tempoMudancaFutura != 0)
    if (tempoAtual >= tempoMudancaFutura) {
      tempoMudancaFutura = 0;
      contOdoEsq++;
      digitalWrite(TESTE_ODOMETRIA_ESQ, LOW);
    }
  if (subindo) {
    if (valAtual > valMax)
      valMax = valAtual;
    else if (valMax - valAtual > VARIACAO_IGNORAVEL) {
      subindo = false;
      valMin = valMax;
      goto contagem;
    }
  } else {
    if (valAtual < valMin)
      valMin = valAtual;
    else if (valAtual - valMin > VARIACAO_IGNORAVEL) {
      subindo = true;
      valMax = valMin;
      goto contagem;
    }
  }
  return;
  contagem: {                     //contagem que ocorre apenas quando o tempo medio do meio ciclo anterior é maior que o meio ciclo atual
    contOdoEsq++;
    digitalWrite(TESTE_ODOMETRIA_ESQ, HIGH);
    if (tempoMudancaFutura != 0) {                     // se a contagem pelo tempo ainda nao foi feita e o meio ciclo ja acabou (meio ciclo muito menor que anterior), conta
      contOdoEsq++;
      digitalWrite(TESTE_ODOMETRIA_ESQ, LOW);
    }
    tempoMudancaFutura = tempoAtual + (tempoAtual - tempoInicio) / 2;
    tempoTotal = tempoAtual - tempoInicio;
    tempoInicio = tempoAtual;
  }
  dados[0] = contOdoEsq;
  dados[1] = tempoTotal;
  return;
}

void verificaOdoDir(unsigned long dados[2]) {
  static boolean subindo = true;
  static unsigned long tempoInicio = 0;
  static unsigned long tempoMudancaFutura = 0;
  static int valMin = 0;
  static int valMax = 0;
  unsigned long tempoTotal;
  int valAtual = analogRead(ODOMETRIA_DIR);

  unsigned long tempoAtual = millis();
  if (tempoMudancaFutura != 0)
    if (tempoAtual >= tempoMudancaFutura) {
      tempoMudancaFutura = 0;
      contOdoDir++;
      digitalWrite(TESTE_ODOMETRIA_DIR, LOW);
    }
  if (subindo) {
    if (valAtual > valMax)
      valMax = valAtual;
    else if (valMax - valAtual > VARIACAO_IGNORAVEL) {
      subindo = false;
      valMin = valMax;
      goto contagem;
    }
  } else {
    if (valAtual < valMin)
      valMin = valAtual;
    else if (valAtual - valMin > VARIACAO_IGNORAVEL) {
      subindo = true;
      valMax = valMin;
      goto contagem;
    }
  }
  return;
  contagem: {                     //contagem que ocorre apenas quando o tempo medio do meio ciclo anterior é maior que o meio ciclo atual
    contOdoDir++;
    digitalWrite(TESTE_ODOMETRIA_DIR, HIGH);
    if (tempoMudancaFutura != 0) {                     // se a contagem pelo tempo ainda nao foi feita e o meio ciclo ja acabou (meio ciclo muito menor que anterior), conta
      contOdoDir++;
      digitalWrite(TESTE_ODOMETRIA_DIR, LOW);
    }
    tempoMudancaFutura = tempoAtual + (tempoAtual - tempoInicio) / 2;
    tempoTotal = tempoAtual - tempoInicio;
    tempoInicio = tempoAtual;
  }
  dados[0] = contOdoDir;
  dados[1] = tempoTotal;
  return;
}
