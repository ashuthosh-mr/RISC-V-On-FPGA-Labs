#include <stdint.h>
#include "spi.h"

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

int main(){

int val = 10; 
int dr1, dr2, dr3;
printf("SPI START \n"); 
set_spi(spi_dr1, 0xDEADBCCC);
val = *spi_dr1;
printf("SPI value dr1 %x \n", val); 
set_spi(spi_dr2, 0xDEADBCCC); 
printf("SPI value dr2 %x \n", *spi_dr2); 
set_spi(spi_dr3, 0xDEADBCCC); 
spi_init(); 
spi_tx_rx_start(); 
spi_enable(); 
//spi_notbusy(); 
printf("SPI TRANSMIT DONE\n"); 
//spi_rx_totalbits(); 
//spi_rx_enable(); 
if(spi_rxne_enable()){ 
dr1 = *spi_dr1;
dr2 = *spi_dr2;
dr3 = *spi_dr3;
printf("SPI received value dr1 %x \n", dr1); 
printf("SPI received value dr2 %x \n", dr2); 
printf("SPI received value dr3 %x \n", dr3); 
} 

dr1 = bitExtracted(dr2, 7, 1);
printf("SPI bit extracted value of dr1 %x \n", dr1); 
dr2 = bitExtracted(dr3,12,12);
printf("SPI bit extracted value of dr2 %x \n", dr2); 
val = concat(dr1, dr2);
printf("SPI ADC value %d \n", val); 



printf("SPI init done\n"); 
 
return 0; 
} 
 


