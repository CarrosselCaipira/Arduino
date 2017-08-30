/* APENAS UMAS DESSAS 3 PROXIMAS DIRETIVAS DEVE ESTAR DESCOMENTADA POR VEZ! */
#define NUM_ROBO 0
//#define NUM_ROBO 1
//#define NUM_ROBO 2

//#define DEBUG
//#define RADIOBIDIR //ativa o envio de dados a partir dos robos (radio birecional).

/* APENAS UMAS DESSAS 2 PROXIMAS DIRETIVAS DEVE ESTAR DESCOMENTADA POR VEZ! */
// Opcao disponivel para testarmos os robos com e sem o pid.
//#define PID_OFF
#define PID_ON

#include <PID_v1.h>

//******* Definicao dos pinos e outros macros*************
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

//*** Definicao das constantes do PID para cada robo Begin***
#ifdef PID_ON
  #if NUM_ROBO == 0
    // Motor esquerdo Robo 0
    #define KP_ESQ 4
    #define KI_ESQ 2
    #define KD_ESQ 2
    // Motor direito Robo 0
    #define KP_DIR 4
    #define KI_DIR 2
    #define KD_DIR 2

  #else
    #if NUM_ROBO == 1
      // Motor esquerdo Robo 1
      #define KP_ESQ 4
      #define KI_ESQ 2
      #define KD_ESQ 2
      // Motor direito Robo 1
      #define KP_DIR 4
      #define KI_DIR 2
      #define KD_DIR 2

    #else
      // Motor esquerdo Robo 2
      #define KP_ESQ 4
      #define KI_ESQ 2
      #define KD_ESQ 2
      // Motor direito Robo 2
      #define KP_DIR 4
      #define KI_DIR 2
      #define KD_DIR 2

    #endif
  #endif
#endif
//******* Definicao das constantes do PID End*************

/*********************************************************

Syntax for Function PID:

PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)

Parameters:

- Input: The variable we're trying to control (double)
- Output: The variable that will be adjusted by the pid (double)
- Setpoint: The value we want to Input to maintain (double)
- Kp, Ki, Kd: Tuning Parameters. these affect how the pid will chage the output. (double>=0)
- Direction: Either DIRECT or REVERSE. determines which direction the output
will move when faced with a given error. DIRECT is most common

**********************************************************/

//*********Variaveis do PID  Begin************************
#ifdef PID_ON

// Motor esquerdo
double Input_esq, Output_esq, Setpoint_esq;
PID myPID_esq(&Input_esq, &Output_esq, &Setpoint_esq, KP_ESQ, KI_ESQ, KD_ESQ, DIRECT);

// Motor direito
double Input_dir, Output_dir, Setpoint_dir;
PID myPID_dir(&Input_dir, &Output_dir, &Setpoint_dir, KP_DIR, KI_DIR, KD_DIR, DIRECT);

#endif
//**********************PID_vars End**********************

byte contOdoEsq = 0; //contador de pulsos, zerado a cada inicio de contagem (a cada novo comando)
byte contOdoDir = 0;

void setup() {
  Serial.begin(115200);

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

  //****************PID_config Begin**********************
  #ifdef PID_ON
    myPID_esq.SetOutputLimits(0,255);
    myPID_esq.SetMode(AUTOMATIC);

    myPID_dir.SetOutputLimits(0,255);
    myPID_dir.SetMode(AUTOMATIC);
  #endif
  //****************PID_config End************************
}

byte velDesejadaDir, velDesejadaEsq;
double velCorrigidaEsq, velCorrigidaDir;

void loop(){

  // estabelecendo a velocidade desejada para cada roda
  // multiplicacao por '3' feita para convercao de unidades (pulsos de pwm -> pulsos do odometro).
  // Casting feito pois as Variaveis Setpoint sao do tipo double.
  Setpoint_esq = (double)(velDesejadaEsq * 3);
  Setpoint_dir = (double)(velDesejadaDir * 3);

  /** determina, para a primeira tentativa, que os valores de velocidade
  /** estão perfeitos, evitando um desvio muito grande no momento da calibragem. */
  Input_dir = Setpoint_dir;
  Input_esq = Setpoint_esq;
  for (;;) {

    myPID_dir.Compute();
    myPID_esq.Compute();

    // Output representa pulsos do odometro, logo, divide-se por 3 para achar o valor da velocidade do PWM.
    // Casting feito pois as variaveis velCorrigida sao inteiras (pulsos de PWM).
    velCorrigidaEsq = (int)(Output_esq / 3); // recebe o valor calibrado da velocidade.
    velCorrigidaDir = (int)(Output_dir / 3); // recebe o valor calibrado da velocidade.
    analogWrite(PWM_MOTOR_ESQ, velCorrigidaEsq);
    digitalWrite(DIRECAO_PWM_MOTOR_ESQ_A, HIGH);
    digitalWrite(DIRECAO_PWM_MOTOR_ESQ_B, LOW);
    analogWrite(PWM_MOTOR_DIR, velCorrigidaDir);
    digitalWrite(DIRECAO_PWM_MOTOR_DIR_A, HIGH);
    digitalWrite(DIRECAO_PWM_MOTOR_DIR_B, LOW);

    verificaOdoEsq();
    verificaOdoDir();

    // Recebendo os valores reais de velocidade de cada roda
    // As variaveis Input sao necessariamente do tipo double, por isso o casting.
    Input_dir = (double)contOdoDir;
    Input_esq = (double)contOdoEsq;
  }
}

const int VARIACAO_IGNORAVEL = 100;

inline void verificaOdoEsq() {
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
      #ifdef TESTE_OCILOSCOPIO
      digitalWrite(TESTE_ODOMETRIA_ESQ, LOW); // Para teste com o ociloscópio. Gera uma onda quadrada.
      #endif
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
  contagem: {  // contagem que ocorre apenas quando o tempo medio do meio ciclo anterior é maior que o meio ciclo atual
    contOdoEsq++;
    #ifdef TESTE_OCILOSCOPIO
      digitalWrite(TESTE_ODOMETRIA_ESQ, HIGH); // Para teste com o ociloscópio. Gera uma onda quadrada.
    #endif
    if (tempoMudancaFutura != 0) { // se a contagem pelo tempo ainda nao foi feita e o meio ciclo ja acabou (meio ciclo muito menor que anterior), conta
      contOdoEsq++;
      #ifdef TESTE_OCILOSCOPIO
        digitalWrite(TESTE_ODOMETRIA_ESQ, LOW); // Para teste com o ociloscópio. Gera uma onda quadrada.
      #endif
    }
    tempoMudancaFutura = tempoAtual + (tempoAtual - tempoInicio) / 2;
    tempoInicio = tempoAtual;
  }
  return;
}

inline void verificaOdoDir() {
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
      #ifdef TESTE_OCILOSCOPIO
        digitalWrite(TESTE_ODOMETRIA_DIR, LOW); // Para teste com o ociloscópio. Gera uma onda quadrada.
      #endif
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
    #ifdef TESTE_OCILOSCOPIO
      digitalWrite(TESTE_ODOMETRIA_DIR, HIGH); // Para teste com o ociloscópio. Gera uma onda quadrada.
    #endif
    if (tempoMudancaFutura != 0) { // se a contagem pelo tempo ainda nao foi feita e o meio ciclo ja acabou (meio ciclo muito menor que anterior), conta
      contOdoDir++;
      #ifdef TESTE_OCILOSCOPIO
        digitalWrite(TESTE_ODOMETRIA_DIR, LOW); // Para teste com o ociloscópio. Gera uma onda quadrada.
      #endif
    }
    tempoMudancaFutura = tempoAtual + (tempoAtual - tempoInicio) / 2;
    tempoInicio = tempoAtual;
  }
  return;
}
