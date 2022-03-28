#include <stdint.h>
#include "spi.h"
#include "cypress_data.h"


int main(){
	int burst = 32;
	printf("SPI SRAM START \n"); 
	spi_init();
	printf("SPI init done\n");
	sram_mode(0x00000080);
	int mode = sram_read_mode();
	int addr = 0x0;
	printf("SRAM is in %x \n",mode);

for(int i =0; i<20; i=i+4){
	sram_write(addr,0xaa9955aa*(i+1),burst);
	printf("write data done: %x\n",(0xaa9955aa)*(i+1));
	addr=addr+4;
	}
addr =0x0000000;
for(int i =0; i<1; i++){
	int read = sram_read(addr,20*8);
	printf("sram read done %x:\n",read);
	addr = addr+5;
	}
	return 0; 
} 
 


