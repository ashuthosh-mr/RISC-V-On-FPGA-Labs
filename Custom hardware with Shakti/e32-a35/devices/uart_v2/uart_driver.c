int putchar(int ch)
{
  register char a0 asm("a0") = ch;
  asm volatile ("li t1, 0x11300" "\n\t"	//The base address of UART config registers
        "uart_status_simple: lb a1, 12(t1)" "\n\t"
        "andi a1,a1,0x2" "\n\t"
        "beqz a1, uart_status_simple" "\n\t"
				"sb a0, 4(t1)"  "\n\t"
				:
				:
				:"a0","t1","cc","memory");
  return 0;
}

int getchar()
{
 register char a0 asm("a0");
 register int a1 asm("a1") = 0;
       asm volatile ("li t1, 0x11300" "\n\t" //The base address of UART config registers
           		  	"uart_statusr: lb t2, 12(t1)" "\n\t"
    				"andi t2, t2, 0x8" "\n\t"
	    			"beqz t2, uart_statusr" "\n\t"
                    "lb a0, 8(t1)"  "\n\t"      //The base address of UART data register
                    :
                    :
                    :"a0","t1","t2","cc","memory");


   return a0;
}

// fnuction used to check if UART is empty. Can be used before exiting a function
int is_empty()
{
    asm volatile (
        "uart_end: li t1, 0x11300" "\n\t"	//The base address of UART config registers
        "lb a0, 12(t1)" "\n\t"
        "andi a0, a0, 0x1" "\n\t"
        "beqz a0, uart_end" "\n\t"
				:
				:
				:"a0","t1","cc","memory");
  return 0;
}
