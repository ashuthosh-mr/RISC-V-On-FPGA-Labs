#include <stdint.h>
#include <stdlib.h>
#include "uart.h"

void uart_busy()
{
	uint8_t status = get_uart8(status_reg);
	int i = 0;
	while (!(status & 0x01))
	{  
 	  status = get_uart8(status_reg);
	  i++;
	  if(i>1000) 
	  {
		printf("ERROR");
		break;
	  }
	}
} 	


int main()
{

	uint8_t status;

	//printf("Setting control_reg \n");

	uart_control_reg(0x2, 0x2, 0x1F);  // (stop_bits, parity_mode, char_size) Here it is (1 stop bit, EVEN parity, char_size = 5)
	
	printf("Enabling transmitter interrupt \n");
	//TD_en();
	//TNF_en(); 
	uart_interrupt_en();

	//printf("Getting status of uart \n");
	status = get_uart8(status_reg);
	//printf("The status of uart before setting tx_reg is %x \n", status);
	
	uart_busy();
	set_uart32(tx_reg, 0x7AAA);
	status = get_uart8(status_reg);
	//printf("The status of uart after setting tx_reg is %x \n", status);

	uart_busy();
	//waitfor(1000);
	//printf("The status of uart before 2nd transmission is %x \n", status);
	set_uart32(tx_reg, 0xAB);
	status = get_uart8(status_reg);
	printf("The status of uart after 2nd transmission is %x \n", status);
/*
	//printf("S");
	uart_control_reg(0x0, 0x1,0x3f);
	uart_interrupt_en();
	//uart_busy();
	//set_uart16(delay_reg, 0x2);
	set_uart32(tx_reg, 0xBBBBBAAAAAA);
	uart_busy();
/*
	int i;
	for (i=1; i<256 ; i=i*2)
	{
		set_uart32(tx_reg, i);
		uart_busy();
	}
*/
	//set_uart32(tx_reg, 0xABB);

	//printf("YQRSSDF");

	return 0;
}	

 // Bits [4:3]  : Parity. 00-None, 01- Odd, 10- Even, 11-undefined
                            // Bits [2:1]  : Stop bits.
                                             //00: 1 Stop bit
                                             //01: 1.5 Stop bits
                                             //10: 2 Stop bits
	
