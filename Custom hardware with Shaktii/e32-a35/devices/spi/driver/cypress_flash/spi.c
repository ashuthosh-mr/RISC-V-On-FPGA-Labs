#include <stdint.h>
#include "spi.h"


int main(){

printf("SPI FLASH START \n"); 
spi_init();
//flash_device_id();
flash_write_enable();
flash_erase();
flash_status_register_read();
flash_write_enable();
flash_write();
flash_read();

printf("SPI init done\n"); 
 
return 0; 
} 
 


