#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_interface.h"
#include "espconn.h"
#include <stdio.h>

#include "stm32f4xx.h"

#include "driver/uart.h"

#include "can.h"

#define ELM_PORT 35000
//#define espconn_send_string(conn, x) espconn_send(conn, x, strlen(x))
#define elm_strcmp(dat, x) memcmp(dat, x, strlen(x))
#define dbg_msg(x) uart0_tx_buffer(x, strlen(x))
#define byteToInt(x,i) (x[i]|(uint32_t)x[i+1]<<8|(uint32_t)x[i+2]<<16|(uint32_t)x[i+3]<<24)
#define asciiTobyte(c) (c>0x39?(c>0x46?c-0x61:c-0x41):c-0x30)

#define ELM_RCV_MAX_SIZE    0x44
#define ELM_TX_MAX_SIZE     128
static struct espconn elm_conn;
static esp_tcp elm_proto;
char elm_resp[ELM_TX_MAX_SIZE];
uint32_t elm_recvData[ELM_RCV_MAX_SIZE] = {0};
static int trans_flag = 0;
static char dev_id[] = "4100BE3FA813";
static char last_cmd[ELM_RCV_MAX_SIZE] = {0};

// Various constants.
const char ELM_OK[] = "OK\r\r>";
const char ELM_NA[] = "NA\r\r>";

struct elm_flags {
  char elm_echo;
  char elm_memory;
  char elm_linefeed;
  char elm_spaces;
  char elm_headers;
  char elm_adaptive;
  char elm_protocol;
  char elm_auto_proto;
}elm_flag = {'0','1','1','1','1','1','0','1'};


struct obd_flags {
  char obd_long_messages;
}obd_flag = {'0'};

struct can_flags {
  char can_monitor;
  char can_extended_addr;
}can_flag = {'0', '0'};

bool is_digit(char c) {
    if (c >= '0' && c <= '9')
        return true;
}

bool is_hex(char c) {
    if ((c >= 'A' && c <= 'F') ||
        (c >= 'a' && c <= 'f') )
        return true;
    else return false;
}

char to_upper(char c) {
    if (is_digit(c))
        return c; 
    if (c < 'A')
        c = c - 'a' + 'A';
    return c;
}

//Converts one character to int from hex upper case or lower case
int chex2int(char c) {
    int val = -1;

    if (is_digit(c)) 
        val = c - '0';
    else if (is_hex(c))
        val = to_upper(c) - 'A' + 10;

    return val;
}

//TODO: handle both hhh and hh hh hh hh formats
int shex2int(char *s) {
    int res = 0;
    for (int i=0; i<strlen(s); i++) {
        int val = 0;
        val = chex2int(s[i]);
        if (val >= 0)
            res = (res << 4) + val; 
    }
}


void espconn_send_string(struct espconn *conn, char *str) {
    char rsp_buf[ELM_TX_MAX_SIZE];
    int offset = 0;
    //Copy the input to output to implement echo
    if (elm_flag.elm_echo == '1'){
        strcpy(rsp_buf, elm_recvData);
        offset = strlen(elm_recvData);
    }
    strcpy(rsp_buf + offset, elm_resp);
    espconn_send(conn, rsp_buf, strlen(rsp_buf));
}


//Some code is insipired by https://github.com/robots/STM32/blob/master/can_encoders/can.c


//Sends the data received on the CAN bus onto serial interface emulating elm327
void ICACHE_FLASH_ATTR elm_tx_cb(CAN_FIFOMailBox_TypeDef *can_pkt) {
    if (can_flag.can_monitor == '1') {
        //Copy the data to response buffer
        *((uint32_t *)(elm_resp + 0                    )) = can_pkt->RDLR;
        *((uint32_t *)(elm_resp + sizeof(can_pkt->RDLR))) = can_pkt->RDHR;

        //Send the data
        espconn_send_string(&elm_conn, elm_resp);
    }
}


//calclates the values of filter and mask based on the CAN msg format
//TODO: add support for extented CNA frames
void rid2filter(int rid, int *filter,  int *mask) {
    *filter = (0x0000FF00) && rid << 8;
    *mask = 0x0000FF00;
}

//calclates the values of filter and mask based on the CAN msg format
//TODO: add support for extented CNA frames
void tid2filter(int tid, int *filter,  int *mask) {
    *filter = (0x000000FF) && tid;
    *mask = 0x000000FF;
}


static void ICACHE_FLASH_ATTR elm_rx_cb(void *arg, char *data, uint16_t len) {
  struct espconn *conn = (struct espconn *)arg;

  //Any activity on the input bus (UART) stops the CAN bus monitoring
  can_flag.can_monitor = 0;

  if (elm_strcmp(data, "AT") == 0) {
    int hhh = shex2int(&(data[3]));
    int filter;
    int mask;

    data += 2;
    memset(elm_resp, 0, 128);
    if (elm_strcmp(data, "Z\r") == 0) {
      can_init(CAN1);
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
//General commands:
    } else if (elm_strcmp(data, "@1\r") == 0) { //display the device description
      strcpy(elm_resp,"OBDII to RS232 Interpreter\r\r>");
    } else if (elm_strcmp(data, "@2\r") == 0) { //display the device identifier
      strcpy(elm_resp,dev_id);
    } else if (elm_strcmp(data, "@3") == 0) {   //store the device identifier
      strcpy(dev_id,&data[2]);
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "\r") == 0) {   //repeat the last command
      strcpy(data,last_cmd);
      elm_rx_cb(arg, data, len); //recursion
    } else if (elm_strcmp(data, "BRD") == 0) {   //Try baud rate divisor hh
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "BRT") == 0) {   //Set Baud Rate handshake timeout hh
      strcpy(elm_resp,ELM_NA);
//OBD commands:
    } else if (elm_strcmp(data, "AL\r") == 0) {   //allow long messages
      obd_flag.obd_long_messages = '1';
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "NL\r") == 0) {   //disable long messages
      obd_flag.obd_long_messages = '0';
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "AR\r") == 0) {   //Automatically receive
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "AT0\r") == 0) {   //Adaptive timing off
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "AT1\r") == 0) {   //Adaptive timing auto1
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "AT2\r") == 0) {   //Adaptive timing auto2
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "BD\r") == 0) {   //Perform a buffer dump
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "BI\r") == 0) {   //Bypass the initialization sequence
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "MA\r") == 0) {   //Monitor all
      //Clear all filters:
      can_filters_clear(CAN1);
      //Start monitoring
      can_flag.can_monitor = '1';
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "MR") == 0) {   //Monitor messages with receiver id hh
      hhh = shex2int(&(data[2]));
      rid2filter(hhh, &filter, &mask);
      //Start monitoring
      can_flag.can_monitor = '1';
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "MT") == 0) {   //Monitor messages with transmitter id hh
      hhh = shex2int(&(data[2]));
      tid2filter(hhh, &filter, &mask);
      //Start monitoring
      can_flag.can_monitor = '1';
      strcpy(elm_resp,ELM_OK);
//CAN commands:
    } else if (elm_strcmp(data, "CAF0\r") == 0) {   //CAN automatic formatting off
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "CAF1\r") == 0) {   //CAN automatic formatting on
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "CEA\r") == 0) {   //turn off CAN extended addressing
        can_flag.can_extended_addr = '0';
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "CEA") == 0) {   //use CAN extended address hh
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "CF") == 0) {   //set the id filter to hh hh hh hh or hhh
      can_set_filter(CAN1, shex2int(&(data[2])), -1); 
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "CFC0") == 0) {   //CAN flow control off
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "CFC1") == 0) {   //CAN flow control on
      strcpy(elm_resp,ELM_NA);
    } else if (elm_strcmp(data, "CM") == 0) {   //Set CAN ID mask to hhh or hh hh hh hh 
        can_set_filter(CAN1, -1, shex2int(&(data[2]))); 
        strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "CRA\r") == 0) {   //Reset CAN receive filters
        can_filters_clear(CAN1);
        strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "CRA") == 0) {   //Set CAN receive address to hhh or hh hh hh hh
        hhh = shex2int(&(data[3]));
        rid2filter(hhh, &filter, &mask);
        can_set_filter(CAN1, filter, mask); 
        strcpy(elm_resp,ELM_OK);

    } else if (elm_strcmp(data, "DPN") == 0) {
      elm_resp[0] = elm_flag.elm_protocol;
      strcat(elm_resp,"\r\r>");
    } else if (elm_strcmp(data, "SPA") == 0) { //Set protocol to Auto, start with h
      elm_flag.elm_auto_proto = '1'; 
      elm_flag.elm_protocol = data[3];
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "SP") == 0) { //Set protocol to h
      elm_flag.elm_auto_proto = '0'; 
      elm_flag.elm_protocol = data[2];
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "SP00") == 0) { //Set protocol to Auto
      elm_flag.elm_auto_proto = '1'; 
      strcpy(elm_resp,ELM_OK);
    } else if (elm_strcmp(data, "TPA") == 0) {  //Try protocol h with auto search
      elm_flag.elm_auto_proto = '1'; 
      strcpy(elm_resp,ELM_OK);
    }
    espconn_send_string(&elm_conn, elm_resp);
    strcpy(last_cmd, data);
  } else if(elm_strcmp(data, "01") == 0) {
      memset(elm_resp, 0, 128);
      switch(asciiTobyte(data[2]<<4)|asciiTobyte(data[3])) {
        case 0:
          strcpy(elm_resp,"4100BE3FA813\r");
        break;
     }
    espconn_send_string(&elm_conn, elm_resp);
  } else { //pass the command to the CAN bus
      usb_cb_ep3_out(data, 1, 0);
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

