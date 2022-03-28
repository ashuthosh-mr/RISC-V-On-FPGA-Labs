#include<stdlib.h>

#define SPI_CR1 	 0x00020000
#define SPI_CR2 	 0x00020004
#define SPI_SR       0x00020008
#define SPI_DR1  	 0x0002000C
#define SPI_DR2  	 0x00020010
#define SPI_DR3  	 0x00020014
#define SPI_CRCPR    0x00020018
#define SPI_RXCRCR   0x0002001C
#define SPI_TXCRCR   0x00020020

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
#define SPI_TOTAL_BITS_RX(x)  	(x << 23)

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

int flash_write_enable(){
	printf("Cypress write enable \n");
	set_spi(spi_dr1, 0x06000000);
	set_spi(spi_dr3, 0x06);
	set_spi(spi_cr1, (SPI_BR(7)|SPI_TOTAL_BITS_TX(8)|SPI_TOTAL_BITS_RX(0)|SPI_SPE|SPI_CPHA|SPI_CPOL));
	waitfor(20);
	spi_notbusy();
	return 1;
}

int flash_command(int command){
	set_spi(spi_dr3, command);
	set_spi(spi_cr1, (SPI_BR(7)|SPI_TOTAL_BITS_TX(8)|SPI_TOTAL_BITS_RX(0)|SPI_SPE|SPI_CPHA|SPI_CPOL));
	waitfor(20);
	spi_notbusy();
	return 1;
}

int flash_cmd_addr(int command, int addr){
	int address1 = bitExtracted(addr, 24, 9);
	int address2 = bitExtracted(addr, 8, 1);
	int data1 = command | address1 ;
	address2 = address2 << 24;
	printf("Erase dr1 \n");
	bin(data1);
	printf("\n");
	set_spi(spi_dr1, data1);
	set_spi(spi_dr2, address2);
	set_spi(spi_dr3, 0);
	set_spi(spi_cr1, (SPI_BR(7)|SPI_TOTAL_BITS_TX(40)|SPI_TOTAL_BITS_RX(0)|SPI_SPE|SPI_CPHA|SPI_CPOL));
	waitfor(20);
	spi_notbusy();
	return 1;
}

int flash_cmd_addr_read(int command, int addr){
	int address1 = bitExtracted(addr, 24, 9);
	int address2 = bitExtracted(addr, 8, 1);
	int cmd_addr = command  | address1;
	int dr2, dr3;
	address2 = address2 << 24;
	set_spi(spi_dr1, cmd_addr);
	set_spi(spi_dr2, address2);
	set_spi(spi_dr3, 0);
	spi_tx_rx_start();	
	set_spi(spi_cr1, (SPI_BR(7)|SPI_TOTAL_BITS_TX(48)|SPI_TOTAL_BITS_RX(32)|SPI_SPE|SPI_CPHA|SPI_CPOL));
	if(spi_rxne_enable()) {
		dr3 = *spi_dr3;
		dr2 = *spi_dr2;
	}
	return dr3;
}

void flash_cmd_addr_data(int command, int addr, int data){
	int address1 = bitExtracted(addr, 24, 9);
	int address2 = bitExtracted(addr, 8, 1);
	int cmd_addr = command  | address1;
	address2 = address2 << 24;
	int data1 = bitExtracted(data, 24, 9);
	data1 = address2 | data1;
	printf("concat of 17bit data and 7dummy \n");
	bin(data1);
	printf("\n");
	int data2 = bitExtracted(data, 8, 1);
	data2 = data2 << 24;
	printf("concat of 17bit data and 7dummy 8 bit of address \n");
	bin(data2);
	printf("\n");
	set_spi(spi_dr1, cmd_addr);
	set_spi(spi_dr2, data1);
	set_spi(spi_dr3, data2);
	set_spi(spi_cr1, (SPI_BR(7)|SPI_TOTAL_BITS_TX(72)|SPI_TOTAL_BITS_RX(0)|SPI_SPE|SPI_CPHA|SPI_CPOL));
	waitfor(20);
	spi_notbusy();
}

void flash_write(){
	printf("Writing to flash\n");
	flash_cmd_addr_data(0x12000000, 0x0, 0x00000005);
	printf("Write done\n");
}

void flash_read(){
	printf("Read from flash\n");
	int dat = flash_cmd_addr_read(0x0C000000, 0x0);
	printf("Read data %x\n",dat);
}	

int flash_cmd_read(int command){
	int dr1, dr2, dr3;
	set_spi(spi_dr1, command);
	set_spi(spi_dr3, command);
	spi_tx_rx_start();
	set_spi(spi_cr1, (SPI_BR(7)|SPI_TOTAL_BITS_TX(8)|SPI_TOTAL_BITS_RX(32)|SPI_SPE|SPI_CPHA|SPI_CPOL));
	if(spi_rxne_enable()) {
		dr3 = *spi_dr3;
		dr2 = *spi_dr2;
	}
  	return dr3;
}

void flash_erase(){
	printf("Cypress erase \n");
	flash_cmd_addr(0x21000000, 0x0);
	printf("Cypress erase done\n");
}

int flash_status_register_read(){
	int stat = 0x3;
	while (stat & 0x03){
		stat = flash_cmd_read(0x05000000);
		printf("flash status register val %x\n", stat);
	}
	return 0;
}
	
	
int flash_device_id(){
	int dr1, dr2, dr3;
	int val1, val2;
	flash_write_enable();
	set_spi(spi_dr1, 0x9f);
	set_spi(spi_dr3, 0x9f);
	spi_tx_rx_start();
	set_spi(spi_cr1, (SPI_BR(7)|SPI_TOTAL_BITS_TX(8)|SPI_TOTAL_BITS_RX(32)|SPI_SPE|SPI_CPHA|SPI_CPOL));
	if(spi_rxne_enable()) {
		dr3 = *spi_dr3;
		dr2 = *spi_dr2;
	}
	
	val1 = bitExtracted(dr3, 24, 9);
	val2 = bitExtracted(dr2, 8, 1);
	dr1 = concat(val2, val1);	
	printf("Device ID %x \n", dr3);
	printf("Device ID %x \n", dr2);
	printf("extracted device id %x \n",dr1);


	return 1;	
}
