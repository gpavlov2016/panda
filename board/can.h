void can_init(CAN_TypeDef *CAN, int silent) {
  set_can_enable(CAN, 1);

  CAN->MCR = CAN_MCR_TTCM | CAN_MCR_INRQ;
  while((CAN->MSR & CAN_MSR_INAK) != CAN_MSR_INAK);

  // http://www.bittiming.can-wiki.info/
  // PCLK = 24 MHz
  uint32_t pclk = 24000;
  uint32_t num_time_quanta = 16;

  // 500 kbps
  uint32_t prescaler = pclk / num_time_quanta / 500;

  // seg 1: 13 time quanta, seg 2: 2 time quanta
  CAN->BTR = (CAN_BTR_TS1_0 * 12) |
    CAN_BTR_TS2_0 | (prescaler - 1);

  // silent loopback mode for debugging
  #ifdef CAN_LOOPBACK_MODE
    CAN->BTR |= CAN_BTR_SILM | CAN_BTR_LBKM;
  #endif

  if (silent) {
    CAN->BTR |= CAN_BTR_SILM;
  }

  // reset
  CAN->MCR = CAN_MCR_TTCM;

  #define CAN_TIMEOUT 1000000
  int tmp = 0;
  while((CAN->MSR & CAN_MSR_INAK) == CAN_MSR_INAK && tmp < CAN_TIMEOUT) tmp++;

  if (tmp == CAN_TIMEOUT) {
    set_led(LED_BLUE, 1);
    puts("CAN init FAILED!!!!!\n");
  } else {
    puts("CAN init done\n");
  }

  // accept all filter
  CAN->FMR |= CAN_FMR_FINIT;

  // no mask
  CAN->sFilterRegister[0].FR1 = 0;
  CAN->sFilterRegister[0].FR2 = 0;
  CAN->sFilterRegister[14].FR1 = 0;
  CAN->sFilterRegister[14].FR2 = 0;
  CAN->FA1R |= 1 | (1 << 14);

  CAN->FMR &= ~(CAN_FMR_FINIT);

  // enable all CAN interrupts
  CAN->IER = 0xFFFFFFFF;
  //CAN->IER = CAN_IER_TMEIE | CAN_IER_FMPIE0 | CAN_IER_FMPIE1;
}

// CAN error
void can_sce(CAN_TypeDef *CAN) {
  #ifdef DEBUG
    if (CAN==CAN1) puts("CAN1:  ");
    if (CAN==CAN2) puts("CAN2:  ");
    #ifdef CAN3
      if (CAN==CAN3) puts("CAN3:  ");
    #endif
    puts("MSR:");
    puth(CAN->MSR);
    puts(" TSR:");
    puth(CAN->TSR);
    puts(" RF0R:");
    puth(CAN->RF0R);
    puts(" RF1R:");
    puth(CAN->RF1R);
    puts(" ESR:");
    puth(CAN->ESR);
    puts("\n");
  #endif

  // clear
  //CAN->sTxMailBox[0].TIR &= ~(CAN_TI0R_TXRQ);
  CAN->TSR |= CAN_TSR_ABRQ0;
  //CAN->ESR |= CAN_ESR_LEC;
  //CAN->MSR &= ~(CAN_MSR_ERRI);
  CAN->MSR = CAN->MSR;
}

int can_cksum(uint8_t *dat, int len, int addr, int idx) {
  int i;
  int s = 0;
  for (i = 0; i < len; i++) {
    s += (dat[i] >> 4);
    s += dat[i] & 0xF;
  }
  s += (addr>>0)&0xF;
  s += (addr>>4)&0xF;
  s += (addr>>8)&0xF;
  s += idx;
  s = 8-s;
  return s&0xF;
}

