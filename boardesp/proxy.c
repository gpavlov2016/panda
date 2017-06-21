#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_interface.h"
#include "espconn.h"

#include "driver/spi_interface.h"
#include "driver/uart.h"
#include "crypto/sha.h"

#define min(a,b) \
 ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
   _a < _b ? _a : _b; })

#define max(a,b) \
 ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
   _a > _b ? _a : _b; })

char ssid[32];
static const int pin = 2;

// Structure holding the TCP connection information.
struct espconn tcp_conn;
// TCP specific protocol structure.
esp_tcp tcp_proto;

// interrupt communication on port 1338
struct espconn inter_conn;
esp_tcp inter_proto;

uint32_t sendData[0x14] = {0};
uint32_t recvData[0x40] = {0};

static int ICACHE_FLASH_ATTR __spi_comm(char *dat, int len, uint32_t *recvData, int recvDataLen) {
  SpiData spiData;

  spiData.cmd = 2;
  spiData.cmdLen = 0;
  spiData.addr = NULL;
  spiData.addrLen = 0;

  // float boot pin
  gpio_output_set(0, 0, 0, (1 << 4));

  // manual CS pin
  gpio_output_set(0, (1 << 5), 0, 0);
  memset(sendData, 0xCC, 0x14);

  // send request
  memcpy(((void*)sendData), dat, len);
  spiData.data = sendData;
  spiData.dataLen = 0x14;
  SPIMasterSendData(SpiNum_HSPI, &spiData);

  // give the ST time to be ready, up to 1s
  for (int i = 0;(gpio_input_get() & (1 << 4)) && i < 100000; i++) os_delay_us(10);

  // TODO: handle this better
  if (gpio_input_get() & (1 << 4)) os_printf("ERROR: SPI receive failed\n");

  // blank out recvData
  memset(recvData, 0xBB, 0x44);

  // receive the length
  spiData.data = recvData;
  spiData.dataLen = 4;
  SPIMasterRecvData(SpiNum_HSPI, &spiData);
  int length = recvData[0];

  // got response, 0x40 works, 0x44 does not
  spiData.data = recvData+1;
  spiData.dataLen = recvDataLen;
  SPIMasterRecvData(SpiNum_HSPI, &spiData);

  // clear CS
  gpio_output_set((1 << 5), 0, 0, 0);

  // set boot pin back
  gpio_output_set((1 << 4), 0, (1 << 4), 0);

  return length;
}

int ICACHE_FLASH_ATTR spi_comm(char *dat, int len, uint32_t *recvData, int recvDataLen) {
  // blink the led during SPI comm
  if (GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << pin)) {
    // set gpio low
    gpio_output_set(0, (1 << pin), 0, 0);
  } else {
    // set gpio high
    gpio_output_set((1 << pin), 0, 0, 0);
  }

  return __spi_comm(dat, len, recvData, recvDataLen);
}

static void ICACHE_FLASH_ATTR tcp_rx_cb(void *arg, char *data, uint16_t len) {
  // nothing too big
  if (len > 0x14) return;

  // do the SPI comm
  spi_comm(data, len, recvData, 0x40);

  espconn_send(&tcp_conn, recvData, 0x44);
}

void ICACHE_FLASH_ATTR tcp_connect_cb(void *arg) {
  struct espconn *conn = (struct espconn *)arg;
  espconn_set_opt(&tcp_conn, ESPCONN_NODELAY);
  espconn_regist_recvcb(conn, tcp_rx_cb);
}

static volatile os_timer_t some_timer;
void ICACHE_FLASH_ATTR some_timerfunc(void *arg) {
  uint8_t timerRecvData[0x44] = {0};
  uint8_t buf[0x44*0x10];
  int i = 0;
  int j;

  while (i < 0x40) {
    int len = spi_comm("\x01\x00\x00\x00", 4, timerRecvData, 0x40);
    if (len == 0) break;

    // if it sends it, assume it's valid CAN
    for (j = 0; j < len; j += 0x10) {
      memcpy(buf + i*0x10, (timerRecvData+4)+j, 0x10);
      i++;
    }
  }

  if (i != 0) {
    espconn_send(&inter_conn, buf, i*0x10);
  }
}

volatile int timer_started = 0;

void ICACHE_FLASH_ATTR inter_disc_cb(void *arg) {
  if (timer_started) {
    os_timer_disarm(&some_timer);
    timer_started = 0;
  }
}

void ICACHE_FLASH_ATTR inter_connect_cb(void *arg) {
  struct espconn *conn = (struct espconn *)arg;
  espconn_set_opt(&inter_conn, ESPCONN_NODELAY);
  espconn_regist_disconcb(conn, inter_disc_cb);

  // setup timer at 200hz
  // TODO: disable when it runs out
  if (!timer_started) {
    os_timer_setfn(&some_timer, (os_timer_func_t *)some_timerfunc, NULL);
    os_timer_arm(&some_timer, 5, 1);
    timer_started = 1;
  }
}

void ICACHE_FLASH_ATTR wifi_init() {
  // default ssid and password
  memset(ssid, 0, 32);
  os_sprintf(ssid, "panda-%08x-BROKEN", system_get_chip_id()); 
  char password[] = "testing123";

  // fetch secure ssid and password
  // update, try 3 times
  for (int i = 0; i < 3; i++) {
    uint8_t digest[SHA_DIGEST_SIZE];
    char resp[0x20];
    __spi_comm("\x00\x00\x00\x00\x40\xD0\x00\x00\x00\x00\x20\x00", 0xC, recvData, 0x40);
    memcpy(resp, recvData+1, 0x20);

    SHA_hash(resp, 0x1C, digest);
    if (memcmp(digest, resp+0x1C, 4) == 0) {
      // OTP is valid
      memcpy(ssid+6, resp, 0x10);
      memcpy(password, resp+0x10, 10);
      break;
    }
    os_delay_us(50000);
  }

  // start wifi AP
  wifi_set_opmode(SOFTAP_MODE);
  struct softap_config config;
  wifi_softap_get_config(&config);
  strcpy(config.ssid, ssid); 
  strcpy(config.password, password);
  config.ssid_len = strlen(ssid);
  config.authmode = AUTH_WPA2_PSK;
  config.beacon_interval = 100;
  config.max_connection = 10;
  wifi_softap_set_config(&config);

  //set IP
  wifi_softap_dhcps_stop(); //stop DHCP before setting static IP
  struct ip_info ip_config;
  IP4_ADDR(&ip_config.ip, 192, 168, 0, 10);
  IP4_ADDR(&ip_config.gw, 0, 0, 0, 0);
  IP4_ADDR(&ip_config.netmask, 255, 255, 255, 0);
  wifi_set_ip_info(SOFTAP_IF, &ip_config);
  wifi_softap_dhcps_start();

  // setup tcp server
  tcp_proto.local_port = 1337;
  tcp_conn.type = ESPCONN_TCP;
  tcp_conn.state = ESPCONN_NONE;
  tcp_conn.proto.tcp = &tcp_proto;
  espconn_regist_connectcb(&tcp_conn, tcp_connect_cb);
  espconn_accept(&tcp_conn);
  espconn_regist_time(&tcp_conn, 60, 0); // 60s timeout for all connections

  // setup inter server
  inter_proto.local_port = 1338;
  inter_conn.type = ESPCONN_TCP;
  inter_conn.state = ESPCONN_NONE;
  inter_conn.proto.tcp = &inter_proto;
  espconn_regist_connectcb(&inter_conn, inter_connect_cb);
  espconn_accept(&inter_conn);
  espconn_regist_time(&inter_conn, 60, 0); // 60s timeout for all connections
}

#define LOOP_PRIO 2
#define QUEUE_SIZE 1
static os_event_t my_queue[QUEUE_SIZE];
void loop();

void ICACHE_FLASH_ATTR web_init();
void ICACHE_FLASH_ATTR elm327_init();

void ICACHE_FLASH_ATTR user_init()
{
  // init gpio subsystem
  gpio_init();

  // configure UART TXD to be GPIO1, set as output
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_GPIO1); 
  gpio_output_set(0, 0, (1 << pin), 0);

  // configure SPI
  SpiAttr hSpiAttr;
  hSpiAttr.bitOrder = SpiBitOrder_MSBFirst;
  hSpiAttr.speed = SpiSpeed_0_5MHz;
  hSpiAttr.mode = SpiMode_Master;
  hSpiAttr.subMode = SpiSubMode_0;

  // TODO: is one of these CS?
  WRITE_PERI_REG(PERIPHS_IO_MUX, 0x105);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, 2);  // configure io to spi mode
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, 2);  // configure io to spi mode
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, 2);  // configure io to spi mode
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, 2);  // configure io to spi mode
  SPIInit(SpiNum_HSPI, &hSpiAttr);
  //SPICsPinSelect(SpiNum_HSPI, SpiPinCS_1);

  // configure UART TXD to be GPIO1, set as output
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5); 
  gpio_output_set(0, 0, (1 << 5), 0);
  gpio_output_set((1 << 5), 0, 0, 0);

  // uart init
  uart_init(BIT_RATE_115200, BIT_RATE_115200);

  // led init
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
  gpio_output_set(0, (1 << pin), (1 << pin), 0);

  os_printf("hello\n");

  // needs SPI
  wifi_init();

  // support ota upgrades
  elm327_init();
  web_init();

  // jump to OS
  system_os_task(loop, LOOP_PRIO, my_queue, QUEUE_SIZE);
}


void ICACHE_FLASH_ATTR loop(os_event_t *events) {
  system_os_post(LOOP_PRIO, 0, 0);
}

