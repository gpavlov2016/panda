void can_filters_clear();

void can_init(CAN_TypeDef *CAN) {
  // enable CAN busses
  if (CAN == CAN1) {
    #ifdef PANDA
      // CAN1_EN
      GPIOC->ODR &= ~(1 << 1);
    #else
      // CAN1_EN
      GPIOB->ODR |= (1 << 3);
    #endif
  } else if (CAN == CAN2) {
    #ifdef PANDA
      // CAN2_EN
      GPIOC->ODR &= ~(1 << 13);
    #else
      // CAN2_EN
      GPIOB->ODR |= (1 << 4);
    #endif
  #ifdef CAN3
  } else if (CAN == CAN3) {
    // CAN3_EN
    GPIOA->ODR &= ~(1 << 0);
  #endif
  }

  CAN->MCR = CAN_MCR_TTCM | CAN_MCR_INRQ;
  while((CAN->MSR & CAN_MSR_INAK) != CAN_MSR_INAK);

  // PCLK = 24000000, 500000 is 48 clocks
  // from http://www.bittiming.can-wiki.ino/
  CAN->BTR = 0x001c0002;

  // loopback mode for debugging
  #ifdef CAN_LOOPBACK_MODE
    CAN->BTR |= CAN_BTR_SILM | CAN_BTR_LBKM;
  #endif

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

  can_filters_clear();

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

void can_filters_clear() {
    // accept all filter
    CAN->FMR |= CAN_FMR_FINIT;

    // no mask
    CAN->sFilterRegister[0].FR1 = 0;
    CAN->sFilterRegister[0].FR2 = 0;
    CAN->sFilterRegister[14].FR1 = 0;
    CAN->sFilterRegister[14].FR2 = 0;
    CAN->FA1R |= 1 | (1 << 14);

    CAN->FMR &= ~(CAN_FMR_FINIT);
}

void can_set_mask() {
    // accept all filter
    CAN->FMR |= CAN_FMR_FINIT;

    // no mask
    CAN->sFilterRegister[0].FR1 = 0;
    CAN->sFilterRegister[0].FR2 = 0;
    CAN->sFilterRegister[14].FR1 = 0;
    CAN->sFilterRegister[14].FR2 = 0;
    CAN->FA1R |= 1 | (1 << 14);

    CAN->FMR &= ~(CAN_FMR_FINIT);
}

// TODO: add others
//"1M" "800K" "500K" "250K" "125K" "100K" "50K" "20K" "10K"
#define TIMINGS_NUM (sizeof(CAN_Timings)/sizeof(struct can_timing_t))
static const struct can_timing_t CAN_Timings[] = {
	{"1M", 0x03, 0x115 } // 1Mbit CAN speed @ 72mHz internal clock
};

#define FILTER_NUM	14
static CAN_FilterInitTypeDef CAN_Filters[FILTER_NUM] = {
	// Id_H,  Id_L    MskIdH  MskIdL  FIFO Filt# Mode                   Scale                  Active 
	{ 0x0000, 0x0000, 0x0000, 0x0000, 0,   0,    CAN_FilterMode_IdMask, CAN_FilterScale_32bit, ENABLE },
};

struct can_timing_t *CAN_Timing;
static NVIC_InitTypeDef CAN_Int;

uint8_t can_change_baudrate( CAN_HANDLE fd, char* baud)
{
	(void)fd;
	uint8_t i;
	for (i = 0; i < TIMINGS_NUM; i++){
		if (strcmp(CAN_Timings[i].baud, baud) == 0) {
			CAN_Timing = &CAN_Timings[i];
			return 0;
		}
	}
	return 1;
}







//Sets the filter or the mask, assuming 32 bit filter 
//-1 means the value should not change
void can_set_filter(int filter, int mask) {

  uint32_t filter_number_bit_pos = 0;

  filter_number_bit_pos = 0;    //filter 0

  if (filter == -1)
      filter = CAN1->FR1;
  if (mask == -1) 
      mask == CAN1->FR2;

  /* Initialisation mode for the filter */
  CAN1->FMR |= FMR_FINIT;

  /* Filter Deactivation */
  CAN1->FA1R &= ~(uint32_t)filter_number_bit_pos;

   /* 32-bit scale for the filter */
    CAN1->FS1R |= filter_number_bit_pos;
    /* 32-bit identifier or First 32-bit identifier */
    CAN1->FR1 = filter;
    /* 32-bit mask or Second 32-bit identifier */
    CAN1->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR2 = mask;

  /*Id/Mask mode for the filter*/
  CAN1->FM1R &= ~(uint32_t)filter_number_bit_pos;

  /* FIFO 0 assignation for the filter */
  CAN1->FFA1R &= ~(uint32_t)filter_number_bit_pos;

  /* Filter activation */
  CAN1->FA1R |= filter_number_bit_pos;

  /* Leave the initialisation mode for the filter */
  CAN1->FMR &= ~FMR_FINIT;
}

}

void can_filter_apply()
{
	int i;
	// setup can filters
	for (i = 0; i < FILTER_NUM; i++) {
		if (CAN_Filters[i].CAN_FilterActivation == DISABLE) 
			break;
		CAN_FilterInit(&CAN_Filters[i]);
	}
}

void can_filter_add_mask(uint16_t cobid, uint16_t cobid_mask, uint8_t prio)
{
	uint8_t i = 0;

	for (i = 0; i < FILTER_NUM; i++) {
		if (CAN_Filters[i].CAN_FilterActivation == DISABLE)
			break;
	}

	// check limit
	if (i >= FILTER_NUM) 
		return;

	CAN_Filters[i].CAN_FilterActivation = ENABLE;
	CAN_Filters[i].CAN_FilterNumber = i;
	CAN_Filters[i].CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_Filters[i].CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_Filters[i].CAN_FilterFIFOAssignment = (prio > 0)? 1: 0;
    CAN_Filters[i].CAN_FilterIdHigh = cobid << 5;
	CAN_Filters[i].CAN_FilterIdLow = 0x0000;
	CAN_Filters[i].CAN_FilterMaskIdHigh = cobid_mask << 5;
    CAN_Filters[i].CAN_FilterMaskIdLow = 0x0004;
}
