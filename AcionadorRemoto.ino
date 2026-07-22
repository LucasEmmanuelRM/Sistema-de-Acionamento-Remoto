#include <Arduino.h>

// Buzzer
#define BUZZER 14
#define BUZZER_DELAY 1000        // Tempo de alternância no estado armado para
unsigned long buzzer_timer;   

// Acionador
#define IGNITE_PIN      21    // Emite o sinal lógico para o MOSFET, acionando o skib
#define CURRENT_PIN     22    // Emite uma pequena corrente no skib, sem acioná-lo
#define CONTINUITY_PIN  23    // Verifica a continuidade no skib
bool continuity_test;

// Configuração de comunicação
#define RX_2 16
#define TX_2 17
#define BD 9600    // Baudrate

#define PK_SIZE 11     // Tamanho do pacote em bytes
#define COD 0xCD26     // Código de reconhecimento
uint16_t current_seq;  // Valor atual da sequência de pacotes
char msg[4];

#define INTERVALO 50            // Intervalo mínimo entre envios de pacote, em ms
unsigned long ultimo_envio;     // Último envio de pacote, em ms

#define CONT_ESTAGIO 5      // No 5º pacote com o mesmo estágio de seguro ou armado, o estágio do AR muda
#define CONT_IGNICAO 10     // No 10º pacote recebido com o estágio de ignição, AR também entra em ignição
#define SEQ_ALLOW 3         // Valor de tolerância para verificação de pacotes sequênciais (ex.: pkt 121 ao 123 são aceitos se vierem após o 120)
uint8_t cont_pkt;           // Conta quantos pacotes de um mesmo estágio chegaram consecutivamente
uint8_t stg_temp;           // Verificador de estágio para o contador acima

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

// Declaração de funções
uint16_t crc16(const uint8_t*, uint8_t);
void montar_pacote(char*);
void atualizar_estagio();
void receber_pacote();
void testar_continuidade();

// ============================================================================================================================================ //
// ============================================================================================================================================ //

void setup(){
  current_seq = 0;
  cont_pkt = 0;
  ultimo_envio = 0;
  buzzer_timer = 0;
  continuity_test = false;

  estagio = SEGURO;
  stg_temp = SEGURO;
  
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  pinMode(IGNITE_PIN, OUTPUT);
  digitalWrite(IGNITE_PIN, LOW);

  pinMode(CONTINUITY_PIN, INPUT);

  Serial2.begin(BD, SERIAL_8N1, RX_2, TX_2);

}

void loop(){
  receber_pacote();
  atualizar_estagio();

  if (millis() - ultimo_envio >= INTERVALO){
    ultimo_envio = millis();
    montar_pacote(msg);
    Serial2.write((uint8_t *)&enviado, PK_SIZE);
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
  enviado.seq = current_seq;
  enviado.stg = (uint8_t)estagio;
  memcpy(enviado.msg, msg, 4);
  enviado.crc = crc16((uint8_t *)&enviado, 9);
}


// Função de atualizar o estado interno do Acionador Remoto
void atualizar_estagio(){
  switch(estagio){

    case SEGURO:
      digitalWrite(BUZZER, LOW);
      memcpy(msg, "WAIT", 4);
      break;

    case ARMADO:
      testar_continuidade();
      if (!continuity_test){
        estagio = SEGURO;
        break;
      }

      if(millis() - buzzer_timer >= BUZZER_DELAY){
        buzzer_timer = millis();
        digitalWrite(BUZZER, !digitalRead(BUZZER));
      }
      memcpy(msg, "RDY ", 4);
      break;

    case IGNICAO:
      digitalWrite(BUZZER, HIGH);
      memcpy(msg, "FIRE", 4);
      digitalWrite(IGNITE_PIN, HIGH);
  }
}


// Função de receber o pacote da Estação Base
void receber_pacote(){
  if (Serial2.available() < PK_SIZE)
    return;

  Serial2.readBytes((uint8_t *)&recebido, PK_SIZE);

  if (recebido.cod != COD || recebido.seq < current_seq || (recebido.seq - current_seq) > SEQ_ALLOW)
    return;
  
  current_seq = recebido.seq;
  uint16_t check = crc16((uint8_t *)&recebido, 9);

  if (check == recebido.crc){
    if (recebido.stg != stg_temp){
      stg_temp = recebido.stg;
      cont_pkt = 0;
    }
    
    if (recebido.stg != IGNICAO){
      if (++cont_pkt >= CONT_ESTAGIO)
        estagio = (Estagio)recebido.stg;
    }
    else if (++cont_pkt >= CONT_IGNICAO && estagio == ARMADO)
      estagio = (Estagio)recebido.stg;
  }
}

void testar_continuidade(){
  if (!continuity_test){
    digitalWrite(IGNITE_PIN, HIGH);

    if (!digitalRead(CONTINUITY_PIN)){
      continuity_test = true;
      estagio = ARMADO;
    }
    else
      estagio = SEGURO;
    digitalWrite(IGNITE_PIN, LOW);
  }
}