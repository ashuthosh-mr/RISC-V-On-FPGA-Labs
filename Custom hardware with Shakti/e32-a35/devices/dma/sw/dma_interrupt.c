//NOTE: Compile with -O2
#include "dma.h"
#include "encoding.h"
#include <stdint.h>
int dma_flag=0;
unsigned long var_for_context [100];
unsigned long* addr_var_for_context= var_for_context;

#define DMA_INTERRUPTS (DMA_CCR_TEIE|DMA_CCR_TCIE|DMA_CCR_EN)

void waitfor(unsigned int secs) {
	unsigned int time = 0;
	while(time++ < secs);
}

void __attribute__ ((noinline)) generic_ISR()
{
	__asm__ ("addi sp,sp,-48" "\n\t"
  "sd t1, 1*8(sp)" "\n\t" 
  "sd t2, 2*8(sp)" "\n\t"
  "sd t3, 3*8(sp)" "\n\t"
  "sd a5, 4*8(sp)" "\n\t"
  "sd ra, 5*8(sp)" "\n\t"
	"call dma_ISR" "\n\t"
  "ld t1, 1*8(sp)" "\n\t" 
  "ld t2, 2*8(sp)" "\n\t"
  "ld t3, 3*8(sp)" "\n\t"
  "ld a5, 4*8(sp)" "\n\t"
  "ld ra, 5*8(sp)" "\n\t"
	"addi sp,sp,48" "\n\t"
	"mret" );
}

void dma_ISR()
{
	asm volatile("li t1,0x11600" "\n\t"
					"lw t2,0xE0(t1)" "\n\t"		//DMA_ISR
					"andi t2,t2,0x200" "\n\t" //Check if chan3 has raised an interrupt
					"li t3,0x200" "\n\t"			
					"bne t2,t3,label3" "\n\t" //If not, then end
					"li t2,0xFFFFFFF" "\n\t"	//Else, clear all interrupts by writing into DMA_IFCR
					"sw t2,0xE8(t1)" "\n\t"			//by writing into DMA_IFCR
					"sw x0,0x40(t1)" "\n\t"		//Disable chan3 by writing into DMA_CCR3
					"sw x0,0xE8(t1)" "\n\t"			//Clear DMA_IFCR so that subsequent interrupts are raised
					"csrr t1,mstatus" "\n\t"
					"ori t2,t2,0x8" "\n\t"
					"csrw mstatus,t2" "\n\t"
					"li t1,0x11300" "\n\t"
					"li t2,0xa" "\n\t"				//newline
					"sb t2,4(t1)" "\n\t"
					"li t2,0x9" "\n\t"				//tab
					"sb t2,4(t1)" "\n\t"
					"li t2,0x2D" "\n\t"				//hyphen
					"sb t2,4(t1)" "\n\t"
					"li t2,0x2D" "\n\t"				//hyphen
					"sb t2,4(t1)" "\n\t"
					"li t2,0xa" "\n\t"				//newline
					"sb t2,4(t1)" "\n\t"
					"j label2" "\n\t"
					"label3: add t1,x0,x0" "\n\t"
					"add t2,x0,x0" "\n\t"
					"add t3,x0,x0" "\n\t"
					"add x0,x0,x0" "\n\t"
					"label1: j label1" "\n\t"
					"label2: add x0,x0,x0" "\n\t"
					"li t1,0x1" "\n\t"
    			"sw t1, %0" "\n\t"
					:: "mem"(dma_flag));
}

int main()
{
	void (*dma_ISR_ptr)() = &generic_ISR;
	write_csr(mtvec,dma_ISR_ptr);
	unsigned long var1= read_csr(mstatus);
	var1= var1 | 0x8;
  write_csr(mstatus,var1);
	var1= read_csr(mie);
	var1= var1 | 0x800;
  write_csr(mie,var1);
    //printf("Starting DMA mem to mem transfer\n");

	//int a[10]={0xbabe0, 0xbabe1, 0xbabe2, 0xbabe3, 0xbabe4, 0xbabe5, 0xbabe6, 0xbabe7, 0xbabe8, 0xbabe9};
	//int b[10];
	//int c[15]={0xcafe0, 0xcafe1, 0xcafe2, 0xcafe3, 0xcafe4, 0xcafe5, 0xcafe6, 0xcafe7, 0xcafe8, 0xcafe9, 0xcafea, 0xcafeb, 0xcafec, 0xcafed, 0xcafee};
	//int d[15];
	int arr1[1000];
	int arr2[1000];
    
  int *a = arr1;//(int*) 0x80003000; //Input from BRAM
  int *c = arr2;//(int*) 0x80004000; //Input from BRAM

  int i;
  int dum1_4byte=0x000000f0;
  int dum2_4byte=0x000000d1;
  int16_t dum1_2byte = 0x00f0;
  int8_t dum1_byte = 0xf0;

  printf("Address of a: %08x c: %08x",a,c);

	//int burst=0;
  for(int burst = 0; burst < 255; ++burst){

		__asm__("fence\n\t");
  		for(i=0;i<1000;++i) {
  	  	*(a+i)= dum1_4byte+((burst+1)*i*2);
  	    *(c+i)= dum2_4byte+((burst+2)*i);
  	}
  	__asm__("fence\n\t");
		printf("a= %08x c= %08x\n burst= %d",*a,*c,burst);
 
  	  
		*dma_cndtr3= 0xFA0;
		*dma_cmar3= a;
		*dma_cpar3= c;
		//*dma_ifcr=0xFFFFFFF;	//TODO this and the next statement gets optimised
		//#pragma OPTIMIZE OFF
		*dma_ifcr=0x0;
		//#pragma OPTIMIZE ON

		/**dma_cndtr5= 0xFA0;
		*dma_cmar5= c;
		*dma_cpar5= d;
		*dma_ccr5= 0x00086ADF; */
		//*dma_ccr3= 0x00076ADF; 
  	*dma_ccr3= (DMA_CCR_BURST_LEN(burst)|DMA_CCR_USER_READ|DMA_CCR_MEM2MEM|DMA_CCR_PL(2)|DMA_CCR_MSIZE(DMA_FOURBYTE)|DMA_CCR_PSIZE(DMA_FOURBYTE)|DMA_CCR_MINC|DMA_CCR_PINC|DMA_CCR_DIR|DMA_INTERRUPTS);
  	//printf("\t DMA_CCR3 value: %08x\n",*dma_ccr3);
		//wait_for_dma_interrupt();
		while(dma_flag==0) {
			printf("DMA transaction ongoing...\n");
			var1= read_csr(mip);
			printf("mip: %08x\n",var1);
			//waitfor(50);
		}
		printf("Transfer donee\n");
		var1= read_csr(mip);
		printf("mip: %08x\n",var1);
		
  	for(i=0;i<1000;i++){
  		if(*(a+i)!=*(c+i)){
  	  	printf("\tDMA has gone wrong somewhere in copying a: %08x to c: %08x i:%d burst: %d\n",*(a+i),*(c+i),i,burst);
  	    return -1;
  	  }
  	}
		printf("DMA works for burst length: %d\n",burst);
		dma_flag=0;
  }
  printf("\t DMA copy from BRAM to TCM seems to work for all bursts\n");
  return 0;
}
