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
set_spi(spi_cr1, (SPI_BR(7)));
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


void waitfor(unsigned int secs) {
	unsigned int time = 0;
	while(time++ < secs);
}
