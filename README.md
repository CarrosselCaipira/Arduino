# Arduino

Repositório em que se encontram os fontes utilizados para a programação dos robôs.

## Compilando

É necessário ter as seguintes bibliotecas instaladas na IDE para a compitação do código (sketch > Incluir Biblioteca > Gerenciar Biblioteca):
* RF24 - [Autor TRMh20](http://tmrh20.github.io/RF24/);

### Linux

A instalação da IDE pode ser feita através do site [oficial](https://www.arduino.cc/en/Main/Software), a partir de repositórios da sua distribuição ou ainda através do pacote [flatpak](https://flathub.org/apps/details/cc.arduino.arduinoide).

Para que você seja capaz de compilar estes fontes sem a necessidade de evocar a IDE do arduino como `root`, é necessário que usuário pertença necessáriamente ao grupo `dialout`. Existem casos em que é necessário pertencer **também** aos grupos `lock`, `uucp` e `tty`. Para isso, execute o seguinte comando:

`sudo usermod -a  $USER -G dialout,lock,tty,uucp`

Após isso efetue o logout de seu usuário ou reinicie o computador para que estas modificações tenham efeito.

>Observação: Caso ocorra um erro durante a inclusão de seu usuário a um grupo, tente rodar a IDE assim mesmo, pois pode ser que pertencer ao grupo em questão não seja necessário em sua distribuição.
