#define BaudReg		0x00011340
#define TxReg 		0x00011344
#define RxReg		0x00011348
#define StatusReg	0x0001134c
#define DelayReg	0x00011350
#define ControlReg	0x00011354
#define InterruptEn	0x00011358

#ifdef IQC
  #define IQ_cycles 0x0001135C    
#endif 

//Defining StatusReg

#define UART_TD		(1<<0)
#define UART_TNF	(1<<1)
#define UART_RNF 	(1<<2)
#define UART_RNE	(1<<3)
#define UART_PE		(1<<4)
#define UART_ORE	(1<<5)
#define UART_FE		(1<<6)
#define UART_BRE	(1<<7)

//Defining ControlReg

#define UART_CHAR_SIZE(x)		(x<<5)
#define UART_PARITY(x)			(x<<3)
#define UART_STOP(x)			(x<<1)

//Defining InterruptEn

#define UART_TDIE	(1<<0)
#define UART_TNFIE	(1<<1)
#define UART_RNFIE	(1<<2)
#define UART_RNEIE	(1<<3)
#define UART_PEIE	(1<<4)
#define UART_OREIE	(1<<5)
#define UART_FEIE	(1<<6)
#define UART_BREIE	(1<<7)

//pointers to register

uint32_t* baud_reg = (const uint32_t*) BaudReg;
uint32_t* tx_reg = (const uint32_t*) TxReg;
uint32_t* rx_reg = (const uint32_t*) RxReg;
uint32_t* status_reg = (const uint32_t*) StatusReg;
uint32_t* delay_reg = (const uint32_t*) DelayReg;
uint32_t* control_reg = (const uint32_t*) ControlReg;
uint32_t* interrupt_en = (const uint32_t*) InterruptEn;
#ifdef IQC
  uint32_t* iq_cycles = (const uint32_t*) IQ_cycles;
#endif


__attribute__((always_inline))
static inline void set_uart64(uint64_t* addr, uint64_t val)
{
	*addr = val;
}

__attribute__((always_inline))
static inline void set_uart32(uint32_t* addr, uint32_t val)
{
	*addr = val;
}

__attribute__((always_inline))
static inline void set_uart16(uint16_t* addr, uint16_t val)
{
	*addr = val;
}

__attribute__((always_inline))
static inline void set_uart8(uint8_t* addr, uint8_t val)
{
	*addr = val;
}

__attribute__((always_inline))
static inline uint16_t get_uart16(uint16_t* addr)
{
	return *addr;
}


__attribute__((always_inline))
static inline uint8_t get_uart8(uint8_t* addr)
{
	return *addr;
}

void uart_interrupt_en()
{
	set_uart8(interrupt_en, (UART_TDIE));	
}

void uart_control_reg(int stop, int parity, int size)
{
	set_uart16(control_reg, (UART_STOP(stop) | UART_PARITY(parity) | UART_CHAR_SIZE(size) ));
}

void waitfor(unsigned int secs) 
{
	unsigned int time = 0;
	while(time++ < secs);
}












