/***************************************************************************
 * Project           			:  shakti devt board
 * Name of the file	     		:  boot.c
 * Created date			        :  3.10.2019
 * Brief Description of file            :  boot code to copy bare metal appln 
                                           from flash to physical memory.

 Copyright (C) 2019  IIT Madras. All rights reserved.

// check LICENSE.iitm

 ***************************************************************************/

#include "boot.h"

#define CODE_SIZE   131072
#define SPI_CR1     0x00020000
#define SPI_CR2     0x00020004
#define SPI_SR      0x00020008
#define SPI_DR1     0x0002000C
#define SPI_DR2     0x00020010
#define SPI_DR3     0x00020014
#define SPI_DR4	    0x00020018
#define SPI_DR5	    0x0002001C
#define SPI_CRCPR   0x00020020
#define SPI_RXCRCR  0x00020024
#define SPI_TXCRCR  0x00020028

// defining SPI_CR1 register

#define SPI_CPHA		(1 << 0)
#define SPI_CPOL		(1 << 1)
#define SPI_MSTR		(1 << 2)
#define SPI_BR(x)		(x << 3)
#define SPI_SPE  		(1 << 6)
#define SPI_LSBFIRST		(1 << 7)
#define SPI_SSI 		(1 << 8)
#define SPI_SSM			(1 << 9)
#define SPI_RXONLY		(1 << 10)
#define SPI_CRCL		(1 << 11)
#define SPI_CCRCNEXT		(1 << 12)
#define SPI_CRCEN		(1 << 13)
#define SPI_BIDIOE		(1 << 14)
#define SPI_BIDIMODE		(1 << 15)
#define SPI_TOTAL_BITS_TX(x)  	(x << 16)
#define SPI_TOTAL_BITS_RX(x)  	(x << 24)

// defining SPI_CR2 register

#define SPI_RX_IMM_START (1 << 16)
#define SPI_RX_START	 (1 << 15)
#define SPI_LDMA_TX      (1 << 14)
#define SPI_LDMA_RX 	 (1 << 13)
#define SPI_FRXTH	 (1 << 12)
#define SPI_DS(x)	 (x << 8)
#define SPI_TXEIE	 (1 << 7)
#define SPI_RXNEIE	 (1 << 6)
#define SPI_ERRIE	 (1 << 5)
#define SPI_FRF		 (1 << 4)
#define SPI_NSSP	 (1 << 3)
#define SPI_SSOE	 (1 << 2)
#define SPI_TXDMAEN	 (1 << 1)
#define SPI_RXDMAEN	 (1 << 0)

//defining SR register

#define SPI_FTLVL(x)  (x << 11)
#define SPI_FRLVL(x)  (x << 9)
#define SPI_FRE	      (1 << 8)
#define SPI_OVR	      (1 << 6)
#define SPI_MODF      (1 << 5)
#define SPI_CRCERR    (1 << 4)
#define TXE	      (1 << 1)
#define RXNE	      (1 << 0)

//pointers to register

int* spi_cr1 = (int*) SPI_CR1;
int* spi_cr2 = (int*) SPI_CR2;
int* spi_sr  = (int*) SPI_SR ;
int* spi_dr1 = (int*) SPI_DR1 ;
int* spi_dr2 = (int*) SPI_DR2 ;
//int* spi_dr3  = (int*) SPI_DR3 ;
//int* spi_dr4  = (int*) SPI_DR4 ;
int* spi_dr5 = (int*) SPI_DR5 ;
//int* spi_crcpr  = (int*) SPI_CRCPR;
//int* spi_rxcrcr = (int*) SPI_RXCRCR;
//int* spi_txcrcr = (int*) SPI_TXCRCR; 

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

int spi_rxne_enable(){
	int value = 0;
	int count = 0;

	while (!(value & 0x1))
{
		waitfor(100);
		value = get_spi(spi_sr);
		if(count++ > 100)
			return -1;
	}

	return 1;
}

#if 0
int spi_notbusy(){
	int value = 0x80;

	while((value & 0x80)){
		//		waitfor(10);
		//		printf("\n Wait for busy");
		value = get_spi(spi_sr);
	}
	return 1;
}
#endif

void waitfor(unsigned int secs) {

	unsigned int time = 0;

	while(time++ < secs);
}

#if 0
int flash_write_enable(){
	//	printf("Cypress write enable \n");
	set_spi(spi_dr1, 0x06000000);
	set_spi(spi_dr5, 0x06);
	set_spi(spi_cr1, (SPI_BR(7)|SPI_TOTAL_BITS_TX(8)|SPI_TOTAL_BITS_RX(0)|SPI_SPE|SPI_CPHA|SPI_CPOL));
	//	waitfor(20000);
	spi_notbusy();
	//	waitfor(20000);
	return 1;
}
#endif

int flash_cmd_to_read(int command, int addr)
{
	int dr5;

	set_spi(spi_dr1, command  |  (addr >> 8));
	set_spi(spi_dr2, addr << 24);
	set_spi(spi_dr5, 0);
	spi_tx_rx_start();
	set_spi(spi_cr1, (SPI_BR(7)|SPI_TOTAL_BITS_TX(48)|SPI_TOTAL_BITS_RX(32)|SPI_SPE|SPI_CPHA|SPI_CPOL));
	waitfor(20);

	if(spi_rxne_enable()) {
		dr5 = *spi_dr5;
	}
	return dr5;

}

int flash_read(int address)
{
	int read_value = flash_cmd_to_read(0x0c000000,address);

	return read_value;
}

int flash_cmd_read(int command)
{
	int dr1, dr2, dr5;
	set_spi(spi_dr1, command);
	set_spi(spi_dr5, command);
	spi_tx_rx_start();
	set_spi(spi_cr1, (SPI_BR(7)|SPI_TOTAL_BITS_TX(8)|SPI_TOTAL_BITS_RX(32)|SPI_SPE|SPI_CPHA|SPI_CPOL));

	if(spi_rxne_enable()) {
		dr5 = *spi_dr5;
		dr2 = *spi_dr2;
	}
	return dr5;
}

int flash_status_register_read()
{
	int stat = 0x3;

	while (stat & 0x03){
		stat = flash_cmd_read(0x05000000);
	}

	return 0;
}


int flash_device_id()
{
	int dr1, dr2, dr3;
	int val1, val2;

	set_spi(spi_dr1, 0x9f000000);
	set_spi(spi_dr5, 0x9f000000);
	spi_tx_rx_start();
	set_spi(spi_cr1, (SPI_BR(7)|SPI_TOTAL_BITS_TX(8)|SPI_TOTAL_BITS_RX(24)|SPI_SPE|SPI_CPHA|SPI_CPOL));

	if(1 == spi_rxne_enable()) {
		dr3 = *spi_dr5;
		dr2 = *spi_dr2;
	}
	else
	{
		return -1;
	}

	val1 = ( dr3 >> 16 ) & 0xff; //bitExtracted(dr3, 8, 17);
	val2 = dr3 & 0xffff; //bitExtracted(dr3, 16, 1);

 	printf("Device ID %x \n", val1);
 	printf("extracted device id %x \n",val2); 

	return 1;	
}

void jumpToRAM()
{
	printf("\n%s\n","Control transferred to RAM");
	asm volatile("fence.i");
	asm volatile( "li x30, 0x80000000" "\n\t"
			"jr x30" "\n\t"
			:
			:
			:"x30","cc","memory"

		    );
}

void main()
{
	unsigned int read_address;  // read/write from/to this address
	int* bram_address = (int*) 0x80000000;
	int read_value, i=0;
	int size;
	int retval = 0;
	int arch = 0;

	asm volatile(
		     "csrr %[arch], marchid\n"
		     :
		     [arch]
		     "=r"
		     (arch)
		    );

	printf("%s\n\nSHAKTI PROCESSORS \n\n",bootlogo);
	printf("%s\n","Booting... vspi1.0\n");

	printf("Booting on Pinaka! hart 0 \n");
	printf("Pinaka is a SoC build on top of Artix7 35T.\n");
	printf("The core belongs to Shakti E class, 32 bit.\n\n\n");
	printf("Supported ISA: RV32IMACNU.\n");
	printf("Processor Arch ID: %x.\n", arch);

	spi_init();

	if(1 == flash_device_id() )
	{

		read_address = 0x00b00000;

		size = flash_read(read_address);

		//printf("content of address %x is %x ::\n", read_address, size);

		read_address=read_address+4;

		if(size == 0 || size == 0xFFFFFFFF)
		{
			printf("No content to boot \n");
			asm volatile ("ebreak");
		}

		for(i = 0; i < size;i++ )
		{

			read_value= flash_read(read_address);

			*(bram_address) = read_value;

			//printf("read address %x data:  %x      write address: %x data:  %x \n", read_address,read_value, bram_address, *(bram_address));
			
			read_address = read_address+4;
			bram_address++;
			
			if(i%1024 == 0)
			{
				printf(".");
			       //printf("read address %x data:  %x      write address: %x data:  %x :: %d\n", read_address,read_value, bram_address, *(bram_address),i);
			}
		}
	}
	else
	{
		printf("Wrong device id \n");

		asm volatile ("ebreak");
	}

	jumpToRAM();

	asm volatile ("ebreak");
}
