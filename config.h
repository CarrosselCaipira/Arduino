#include <SPI.h>
// #include <nRF24L01.h>
#include <RF24.h> /* By TMRh20 - http://tmrh20.github.io/RF24/ */

namespace Config {
  const long int SERIAL_BIT_RATE = 115200; /* Pode ser definido para 115200 ou 9600. Lembre-se de que este valor deve ser o mesmo do usado pela classe do rádio, encontrada em radio.hpp.e em RX.ino */

  const int BUFFER_SIZE = 9; /* (conferir, parecer estarmos usando 7, 2 por robo e 1 para o 0x80) tamanho do buffer utilizado para armazenar os dados enviados. Deve ser definido igual em RX.ino, TX.ino e em radio.hpp */

  // static const TEMPO_RETRY = 15; /* intervalo de tempo entre cada tentativa de leitura. Definido em multiplos de 250us, portanto 0 = 250us e 15 = 4000us. (us: microsegundos) */
  // static const NUM_RETRY = 15; /* número de tentativas antes de desistir da leitura */

  // Identificador do rádio
  // static const byte chave[6] = { 'U', 'n', 'e', 's', 'p' }; /* Chave para comunicação entre RX e TX. Deve ser a mesma em ambos os códigos. */
  const byte PIPE_CHAVE[6] = {"00001"}; /* Chave para comunicação entre RX e TX. Deve ser a mesma em ambos os códigos. */
  const byte IND_PIPE_LEITURA = 0; /* indica qual pipe será aberta para leitura das informações vindas do rádio. Valores possíveis: [0,5]. 0 e 1 podem usar endereços de 5 bytes, os demais apenas 1. Talvez seja interessante trocar para 1 caso consideremos a comunicação bidirecional pois as escritas ocorrem  nesse pipe. */

  const byte CANAL = 88; /* canal sendo utilizado pelo rádio usado para a comunicação. */

  const byte CARACTERE_INICIAL = 0x80; /* primeiro byte da transmissao - fixo - caso alterado deve ser alterado no código do pc também em radio.hpp. */

}
