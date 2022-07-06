/********************************************************
 * Changes at Telemetry Library:
 * - Serial => to Serial1 and pins 4,16
 * - // my Test for long string
 *   #define INCOMING_BUFFER_SIZE 1000
 *   #define OUTGOING_BUFFER_SIZE 1000
 * 
 * 
 ********************************************************/ 
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_err.h"

#include "Arduino.h"
#include "Telemetry.h"

#include "telemetry_core.h"
#include "telemetry_utils.h"
// #include "greatest.h"
// #include "test.h"

enum tUartProtocolResult {
  uprUnknownError = -128,
  uprTimeoutError,
  uprDataTooBig,
  uprHeaderCrcError,
  uprPayloadCrcError,
  uprUnknownDataReceived,
  uprOK = 1
};

typedef struct  {
  uint32_t headerID;
  uint16_t commandID;
  uint16_t descriptor;
  uint16_t payloadSize;
  uint32_t headerCRC32;
  uint32_t payloadCRC32;
}tUartPacketHeader;

tUartProtocolResult uartSendPacket(tUartPacketHeader *packetHeader, byte *payload, uint16_t payloadSize);

tUartProtocolResult uartReceivePacket(tUartPacketHeader *packetHeader, byte *payload, uint16_t &payloadSize, uint16_t timeoutMs);

/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

#define ECHO_TEST_TXD (4)
#define ECHO_TEST_RXD (16)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (UART_NUM_1)
#define ECHO_UART_BAUD_RATE     (115200)
#define ECHO_TASK_STACK_SIZE    (20000)

#define BUF_SIZE (1024)

static void echo_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Configure a temporary buffer for the incoming data
    char *dataTx = (char *) malloc(BUF_SIZE);
    char *dataRx = (char *) malloc(BUF_SIZE);
    for(int i=0; i<1024; i++){
      dataTx[i] = i;
    }
    for(int i=0; i<32; i++){
      Serial.printf("i=%d 0x%X\n", i, (char*)dataTx[i]);
    }
    int len = 0;
    while (1) {
        vTaskDelay( 5000 * portTICK_PERIOD_MS ); 
        // Write data back to the UART
        Serial.println("Tx-1...");
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) dataTx, 32);
        Serial.println("Tx-2...");
        // Read data from the UART
        len = uart_read_bytes(ECHO_UART_PORT_NUM, dataRx, BUF_SIZE, 20 / portTICK_RATE_MS);
        dataRx[128] = '\0';
        for(int i=0; i<32; i++){
          Serial.printf("i=%d len=%d 0x%X\n", i, len, (char*)dataRx[i]);
        }
    }
}

Telemetry TM;
float thrTx;
float thrRx;

struct TM_state {
  uint8_t called;
  char rcvString[OUTGOING_BUFFER_SIZE];
  char rcvTopic[OUTGOING_BUFFER_SIZE];
};

TM_state state;

void callback_str(TM_state* s, TM_msg* m)
{
  s->called = 1;
  char str[OUTGOING_BUFFER_SIZE] = {0};
  if(emplace(m,str,OUTGOING_BUFFER_SIZE))
  {
    strcpy(s->rcvString,str);
    strcpy(s->rcvTopic,m->topic);
    Serial.printf("[callback_str] topic: %s msg: %s size=%d \n", m->topic, (char*)m->buffer, m->size);
  }
}

void setup() {

  Serial.begin(115200);
  Serial.println("Start...");
  delay(500);

  Serial1.setRxBufferSize(1024);
  TM.begin(115200);


  char longStr[1024];
  for(int i=0; i<1024; i++){
    longStr[i] = (char) i+1;
    if( longStr[i] == 0 ){
      longStr[i] = 1;
    }
  }
  longStr[1000] = '\0';
  Serial.printf("strlen(msg)=%d\n", strlen(longStr)); 

  TM.pub("LONGMSG", longStr);
  // delay(1000);  // wait for a second
  TM.pub_f32("throttle", thrTx);
  TM.attach_f32_to("throttle", &thrRx);

  TM.sub(callback_str, &state);
  // xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);

}

int i;
void loop() {
  thrTx += 1.5;
  TM.pub_f32("throttle", thrTx);
  i++;
  String pubStr = String(i);
  TM.pub("Hello", pubStr.c_str());

  TM.update();

  delay(2000);
  Serial.printf(" thrRx=%f\n", thrRx);
}

