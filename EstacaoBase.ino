#include <Arduino.h>

// LEDs de estágios
#define LED_G 21    // Estágio Seguro
#define LED_Y 22    // Estágio Armado
#define LED_R 23    // Estágio de Ignição

// LED RGB para verificação da comunicação
#define LED_RGB_R 25
#define LED_RGB_G 26
#define LED_RGB_B 27

////////////////////////////////////////////////////////////
//  Piscando, verde    = Sem conexão com o acionador      //
//  Aceso, verde       = Com conexão, acionador em espera //
//  Piscando, amarleo  = Falta de continuidade no skib    //
//  Aceso, amarelo     = Com conexão, pronto pra ignição  //
//  Piscando, vermelho = Ignição em andamento             //
////////////////////////////////////////////////////////////

// Chaves de Estágio
#define KEY_A 32    // Chave de armação (chave seletora)
#define KEY_I 33    // Chave de ignição (botão)

// Configuração de comunicação
#define RX_2 16
#define TX_2 17
#define BD 9600    // Baudrate

#define PK_SIZE 11     // Tamanho do pacote em bytes
#define COD 0xCD26     // Código de reconhecimento
uint16_t current_seq;  // Valor atual da sequência de pacotes
char msg[4];

#define intervalo 50            // Intervalo mínimo entre envios de pacote, em ms
unsigned long ultimo_envio;     // Último envio de pacote, em ms
unsigned long ultimo_contato;   // Último contato com o AR

// Formato do pacote
typedef struct{
  uint16_t cod;
  uint16_t seq; // Valor incremental de pacotes em sequência
  uint8_t stg;  // ID do estágio
  char msg[4];  // Mensagem opcional
  uint16_t crc; // Bytes de verificação CRC-16
}Pacote;

Pacote enviado;
Pacote recebido;

// Máquina de estados interna
enum Estagio{
  SEGURO = 1,
  ARMADO = 2,
  IGNICAO = 3
};

Estagio estagio;

// Máquina de estados do Acionador Remoto
enum EstagioAR{
  SEM_CONEXAO = 0,
  ESPERA = 1,
  PRONTO = 2,
  ACIONADO = 3
};

EstagioAR estagio_ar;

// Declaração de funções
uint16_t crc16(const uint8_t*, uint8_t);
void montar_pacote(char*);
void atualizar_estagio();
void receber_pacote();

// ============================================================================================================================================ //
// ============================================================================================================================================ //

void setup(){
  current_seq = 0;
  ultimo_envio = 0;
  ultimo_contato = 0;

  estagio = SEGURO;
  estagio_ar = SEM_CONEXAO;

  pinMode(LED_G, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_R, OUTPUT);

  pinMode(LED_RGB_R, OUTPUT);
  pinMode(LED_RGB_G, OUTPUT);
  pinMode(LED_RGB_B, OUTPUT);

  pinMode(KEY_A, INPUT);
  pinMode(KEY_I, INPUT_PULLUP);

  Serial2.begin(BD, SERIAL_8N1, RX_2, TX_2);

}

void loop(){
  atualizar_estagio();

  if (millis() - ultimo_envio >= intervalo){
    ultimo_envio = millis();
    montar_pacote(msg);
    Serial2.write((uint8_t *)&enviado, PK_SIZE);
  }

  receber_pacote();

  if(millis() - ultimo_contato > 10*intervalo){
    estagio_ar = SEM_CONEXAO;
  }

}

// ============================================================================================================================================ //
// ============================================================================================================================================ //

// CRC-16/CCITT-FALSE 
uint16_t crc16(const uint8_t *pData, uint8_t size){
    uint8_t i;
    uint16_t wCrc = 0xffff;
    while (size--) {
        wCrc ^= (uint16_t)*pData++ << 8;
        for (i=0; i < 8; i++)
            wCrc = wCrc & 0x8000 ? (wCrc << 1) ^ 0x1021 : wCrc << 1;
    }
    return wCrc & 0xffff;
}


// Função de montar o pacote a ser enviado
void montar_pacote (char* msg){
  enviado.cod = COD;
  enviado.seq = current_seq++;
  enviado.stg = (uint8_t)estagio;
  memcpy(enviado.msg, msg, 4);
  enviado.crc = crc16((uint8_t *)&enviado, 9);
}


// Função de atualizar o estado interno da Estação Base
void atualizar_estagio(){
  if(digitalRead(KEY_A) == HIGH){
    if(digitalRead(KEY_I) == HIGH)
      estagio = ARMADO;
    else
      estagio = IGNICAO;
  }
  else
    estagio = SEGURO;

  switch(estagio){

    case SEGURO:
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_Y, LOW);
      digitalWrite(LED_R, LOW);
      memcpy(msg, "WAIT", 4);
      break;

    case ARMADO:
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_Y, HIGH);
      digitalWrite(LED_R, LOW);
      memcpy(msg, "RDY ", 4);
      break;

    case IGNICAO:
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_Y, HIGH);
      digitalWrite(LED_R, HIGH);
      memcpy(msg, "FIRE", 4);
  }
}


// Função de receber o pacote do Acionador Remoto e atualizar o LED RGB
void receber_pacote(){
  if (Serial2.available() < PK_SIZE)
    return;

  Serial2.readBytes((uint8_t *)&recebido, PK_SIZE);

  if (recebido.cod != COD)
    return;
  
  ultimo_contato = millis();
  uint16_t check = crc16((uint8_t *)&recebido, 9);

  if (check == recebido.crc)
    estagio_ar = (EstagioAR)recebido.stg;

  switch(estagio_ar){
    case SEM_CONEXAO:
      if (millis() - ultimo_envio >= intervalo*2){
        digitalWrite(RGB_R, LOW);
        digitalWrite(RGB_G, digitalRead(RGB_G));
        digitalWrite(RGB_B, LOW);
      }
      break;

    case ESPERA:
      digitalWrite(RGB_R, LOW);
      digitalWrite(RGB_G, HIGH);
      digitalWrite(RGB_B, LOW);
      break;

    case PRONTO:
      if (recebido.msg == "RDY "){
        digitalWrite(RGB_R, HIGH);
        digitalWrite(RGB_G, HIGH);
        digitalWrite(RGB_B, LOW);
      }
      else if (recebido.msg == "NSKB" && millis() - ultimo_envio >= intervalo*2){
        digitalWrite(RGB_R, digitalRead(RGB_R));
        digitalWrite(RGB_G, digitalRead(RGB_G));
        digitalWrite(RGB_B, LOW);
      }
      break;

    case ACIONADO:
      if (millis() - ultimo_envio >= intervalo){
        digitalWrite(RGB_R, digitalRead(RGB_R));
        digitalWrite(RGB_G, LOW);
        digitalWrite(RGB_B, LOW);
      }
  }
}
