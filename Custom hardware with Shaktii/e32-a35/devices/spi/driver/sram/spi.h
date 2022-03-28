#include<stdlib.h>

#define SPI_CR1 	 0x00020000
#define SPI_CR2 	 0x00020004
#define SPI_SR       0x00020008
#define SPI_DR1  	 0x0002000C
#define SPI_DR2  	 0x00020010
#define SPI_DR3  	 0x00020014
#define SPI_DR4		 0x00020018
#define SPI_DR5		 0x0002001C
#define SPI_CRCPR    0x00020020
#define SPI_RXCRCR   0x00020024
#define SPI_TXCRCR   0x00020028

// defining SPI_CR1 register

#define SPI_CPHA				(1 << 0)
#define SPI_CPOL				(1 << 1)
#define SPI_MSTR				(1 << 2)
#define SPI_BR(x)				(x << 3)
#define SPI_SPE  				(1 << 6)
#define SPI_LSBFIRST			(1 << 7)
#define SPI_SSI 				(1 << 8)
#define SPI_SSM					(1 << 9)
#define SPI_RXONLY				(1 << 10)
#define SPI_CRCL				(1 << 11)
#define SPI_CCRCNEXT			(1 << 12)
#define SPI_CRCEN				(1 << 13)
#define SPI_BIDIOE				(1 << 14)
#define SPI_BIDIMODE			(1 << 15)
#define SPI_TOTAL_BITS_TX(x)  	(x << 16)
#define SPI_TOTAL_BITS_RX(x)  	(x << 24)

// defining SPI_CR2 register

#define SPI_RX_IMM_START (1 << 16)
#define SPI_RX_START	 (1 << 15)
#define SPI_LDMA_TX		 (1 << 14)
#define SPI_LDMA_RX		 (1 << 13)
#define SPI_FRXTH		 (1 << 12)
#define SPI_DS(x)		 (x << 8)
#define SPI_TXEIE		 (1 << 7)
#define SPI_RXNEIE		 (1 << 6)
#define SPI_ERRIE		 (1 << 5)
#define SPI_FRF			 (1 << 4)
#define SPI_NSSP		 (1 << 3)
#define SPI_SSOE		 (1 << 2)
#define SPI_TXDMAEN		 (1 << 1)
#define SPI_RXDMAEN		 (1 << 0)

//defining SR register

#define SPI_FTLVL(x)	(x << 11)
#define SPI_FRLVL(x)	(x << 9)
#define SPI_FRE			(1 << 8)
#define SPI_OVR			(1 << 6)
#define SPI_MODF		(1 << 5)
#define SPI_CRCERR		(1 << 4)
#define TXE				(1 << 1)
#define RXNE			(1 << 0)

//pointers to register

int* spi_cr1 = (int*) SPI_CR1;
int* spi_cr2 = (int*) SPI_CR2;
int* spi_sr  = (int*) SPI_SR ;
int* spi_dr1  = (int*) SPI_DR1 ;
int* spi_dr2  = (int*) SPI_DR2 ;
int* spi_dr3  = (int*) SPI_DR3 ;
int* spi_dr4  = (int*) SPI_DR4 ;
int* spi_dr5  = (int*) SPI_DR5 ;
int* spi_crcpr  = (int*) SPI_CRCPR;
int* spi_rxcrcr = (int*) SPI_RXCRCR;
int* spi_txcrcr = (int*) SPI_TXCRCR; 

void set_spi(int* addr, int val)
{
    *addr = val;
}

int get_spi(int* addr)
{
 return *addr;
}

void spi_init(){
set_spi(spi_cr1, (SPI_BR(7)|SPI_CPHA|SPI_CPOL));
}

void spi_tx_rx_start(){
set_spi(spi_cr2, (SPI_RX_IMM_START));
}

void spi_enable(){
set_spi(spi_cr1, (SPI_BR(7)|SPI_TOTAL_BITS_TX(4)|SPI_TOTAL_BITS_RX(16)|SPI_SPE));
}

void spi_rx_enable(){
set_spi(spi_cr2, (SPI_RX_START));
}

int bitExtracted(int number, int k, int p) 
{ 
      return (((1 << k) - 1) & (number >> (p - 1))); 
} 

int concat(int x, int y) {
      unsigned pow = 10;
          while(y >= pow)
                    pow *= 10;
                        return x * pow + y;        
}

int spi_rxne_enable(){
	int value = 0;
	while (!(value & 0x1)){
		waitfor(100);
		value = get_spi(spi_sr);
	}
	return 1;
}

int spi_notbusy(){
	int value = 0x80;
	while((value & 0x80)){
		waitfor(10);
		value = get_spi(spi_sr);
	}
	return 1;
}

void bin(unsigned n) 
{ 
    unsigned i; 
    for (i = 1 << 31; i > 0; i = i / 2) 
        (n & i)? printf("1"): printf("0"); 
} 

void waitfor(unsigned int secs) {
	unsigned int time = 0;
	while(time++ < secs);
}

void sram_cmd_addr_data(int command, int addr, int data,int burst){
	int address1 = bitExtracted(addr, 24, 1);
	int cmd_addr = command  | address1;
	set_spi(spi_dr1, cmd_addr);
	set_spi(spi_dr2, data);
	set_spi(spi_dr5, 0);
	set_spi(spi_cr1, (SPI_BR(7)|SPI_TOTAL_BITS_TX(32+burst)|SPI_TOTAL_BITS_RX(0)|SPI_SPE|SPI_CPHA|SPI_CPOL));
	waitfor(20);
	spi_notbusy();
}

void sram_write(int address, int data, int burst_size){
	sram_cmd_addr_data(0x02000000,address,data,burst_size);
}

int sram_cmd_to_read(int command, int addr,int burst){
	int dr[5];
	int address1 = bitExtracted(addr, 24, 1);
	int cmd_addr = command  | address1;
	set_spi(spi_dr1, cmd_addr);
	set_spi(spi_dr2, 0);
	set_spi(spi_dr5, 0);
	spi_tx_rx_start();
	set_spi(spi_cr1, (SPI_BR(7)|SPI_TOTAL_BITS_TX(40)|SPI_TOTAL_BITS_RX(burst)|SPI_SPE|SPI_CPHA|SPI_CPOL));
	waitfor(20);
	if(spi_rxne_enable()) {
		dr[4] = *spi_dr5;
		dr[3] = *spi_dr4;
		dr[2] = *spi_dr3;
		dr[1] = *spi_dr2;
		dr[0] = *spi_dr1;
	}
	for(int j=1; j<(burst/32)+1;j++){
		int m =4-burst/32+j;
		for(int k=25;k>0;k=k-8){
			printf("Reading Value %x \n",bitExtracted(dr[m],8,k));
			}
		
	}
	//printf("Reading Value: %x %x %x %x %x\n",dr1,dr2,dr3,dr4,dr5);
	return dr[4];
}

int sram_read(int address, int burst){
	sram_cmd_to_read(0x03000000,address,burst);
}

int sram_cmd_read(int command){
	int dr1, dr2, dr5;
	set_spi(spi_dr1, command);
	set_spi(spi_dr5, command);
	spi_tx_rx_start();
	set_spi(spi_cr1, (SPI_BR(7)|SPI_TOTAL_BITS_TX(8)|SPI_TOTAL_BITS_RX(8)|SPI_SPE|SPI_CPHA|SPI_CPOL));
	if(spi_rxne_enable()) {
		dr5 = *spi_dr5;
	}
  	return dr5;
}   
int sram_read_mode(){
	int mode = sram_cmd_read(0x05000000);
	return mode;	
}

void sram_wrmr(int command, int data){
	int data1 = data << 16;
	int cmd_data = command  | data1;
	set_spi(spi_dr1, cmd_data);
	set_spi(spi_dr5, 0);
	set_spi(spi_cr1, (SPI_BR(7)|SPI_TOTAL_BITS_TX(16)|SPI_TOTAL_BITS_RX(0)|SPI_SPE|SPI_CPHA|SPI_CPOL));
	waitfor(20);
	spi_notbusy();
}
                                                                  
void sram_mode(int mode){	//mode: [7:6] bit control the mode 
	sram_wrmr(0x01000000,mode);
}
	


