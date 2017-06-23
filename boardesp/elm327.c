#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_interface.h"
#include "espconn.h"
#include <stdio.h>

#include "driver/uart.h"

#define ELM_PORT 35000
#define espconn_send_string(conn, x) espconn_send(conn, x, strlen(x))
#define elm_strcmp(dat, x) memcmp(dat, x, strlen(x))
#define dbg_msg(x) uart0_tx_buffer(x, strlen(x))
#define byteToInt(x,i) (x[i]|(uint32_t)x[i+1]<<8|(uint32_t)x[i+2]<<16|(uint32_t)x[i+3]<<24)
#define asciiTobyte(c) (c>0x39?(c>0x46?c-0x61:c-0x41):c-0x30)

static struct espconn elm_conn;
static esp_tcp elm_proto;
char elm_resp[128];
uint32_t elm_recvData[0x44] = {0};
static int trans_flag = 0;

// Various constants.
const char ELM_OK[] = "OK\r\r>";

struct elm_flags {
  char elm_echo;
  char elm_memory;
  char elm_linefeed;
  char elm_spaces;
  char elm_headers;
  char elm_adaptive;
  char elm_protocol;
}elm_flag = {'0','1','1','1','1','1','0'};

static void ICACHE_FLASH_ATTR elm_rx_cb(void *arg, char *data, uint16_t len) {
  struct espconn *conn = (struct espconn *)arg;
  if (elm_strcmp(data, "AT") == 0) {
    data += 2;
    memset(elm_resp, 0, 128);
    if (elm_strcmp(data, "Z\r") == 0) {
      strcpy(elm_resp,"ELM327 v2.1\r\r>");
    } else if (data[0] == 'E') {
      elm_flag.elm_echo = data[1];
      strcpy(elm_resp,ELM_OK);
    } else if (data[0] == 'M') {
      elm_flag.elm_memory = data[1];
      strcpy(elm_resp,ELM_OK);
    } else if (data[0] == 'L') {
      elm_flag.elm_linefeed = data[1];
      strcpy(elm_resp,ELM_OK);
    } else if (data[0] == 'S') {
      elm_flag.elm_spaces = data[1];
      strcpy(elm_resp,ELM_OK);
    } else if (data[0] == 'H') {
      elm_flag.elm_headers = data[1];
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "AT") == 0) {
      elm_flag.elm_adaptive = data[2];
      strcpy(elm_resp,ELM_OK);
    } else if (data[0] == 'I') {
      strcpy(elm_resp,"ELM327 v2.1\r\r>");
    } else if (elm_strcmp(data, "@1\r") == 0) {
      strcpy(elm_resp,"OBDII to RS232 Interpreter\r\r>");
    } else if (elm_strcmp(data, "DPN") == 0) {
      elm_resp[0] = elm_flag.elm_protocol;
      strcat(elm_resp,"\r\r>");
    } else if (elm_strcmp(data, "SP") == 0) {
      elm_flag.elm_protocol = data[2];
      strcpy(elm_resp,ELM_OK);
    }
    espconn_send_string(&elm_conn, elm_resp);
  } else if(elm_strcmp(data, "01") == 0) {
      memset(elm_resp, 0, 128);
      switch(asciiTobyte(data[2]<<4)|asciiTobyte(data[3])) {
        case 0:
          strcpy(elm_resp,"4100BE3FA813\r");
        break;
     }
    espconn_send_string(&elm_conn, elm_resp);
  }
  uart0_tx_buffer(elm_resp, strlen(elm_resp));
}

void ICACHE_FLASH_ATTR elm_tcp_connect_cb(void *arg) {
  struct espconn *conn = (struct espconn *)arg;
  espconn_set_opt(&elm_conn, ESPCONN_NODELAY);
  espconn_regist_recvcb(conn, elm_rx_cb);
}

void ICACHE_FLASH_ATTR elm327_init() {
  // control listener
  elm_proto.local_port = ELM_PORT;
  elm_conn.type = ESPCONN_TCP;
  elm_conn.state = ESPCONN_NONE;
  elm_conn.proto.tcp = &elm_proto;
  espconn_regist_connectcb(&elm_conn, elm_tcp_connect_cb);
  espconn_accept(&elm_conn);
}

