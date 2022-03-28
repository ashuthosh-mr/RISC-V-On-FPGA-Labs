#include "qspi.h"
#include "cypress_data.h"

#define WRITEB 10
#define ERASE WRITEB/4
#define MEM_TYPE_N25Q256_ID 0x20BA1910
char fail_bit = 0;
int status = 0;

int check_fail_bit(){
  if(fail_bit){
        fail_bit = 0;
        return -1;
  }
    else{
        fail_bit = 0;
        return 0;
    }
}

int pageProgramSingleSPI(int value1, int value2, int value3, int value4, int address){
    if(micron_write_enable(status)){
        printf("Panic: Write Enable Command Failed to execute");
        return -1;
    }
    reset_interrupt_flags(); 
   
    set_qspi_shakti32(dlr,DL(16));
//  set_qspi_shakti32(ccr,(CCR_DDRM | CCR_FMODE(CCR_FMODE_INDWR)|CCR_DMODE(SINGLE)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(SINGLE)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0x12))); 
    set_qspi_shakti32(ccr,(CCR_FMODE(CCR_FMODE_INDWR)|CCR_DMODE(SINGLE)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(SINGLE)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0x12))); 
    set_qspi_shakti32(ar, address);
    set_qspi_shakti32(dr,value1);
    set_qspi_shakti32(dr,value2);
    set_qspi_shakti32(dr,value3);
    set_qspi_shakti32(dr,value4);
    waitfor(150); 
    wait_for_tcf(status);
    reset_interrupt_flags();
    return wait_for_wip(); // Function which checks if WIP is done, indicating completion of Page Program
}

int pageProgramQuadSPI(int value1, int value2, int value3, int value4, int address){
    if(micron_write_enable(status)){
        printf("Panic: Write Enable Command Failed to execute");
        return -1;
    }
    reset_interrupt_flags(); 
   
    set_qspi_shakti32(dlr,DL(16));
//  set_qspi_shakti32(ccr,(CCR_DDRM | CCR_FMODE(CCR_FMODE_INDWR)|CCR_DMODE(SINGLE)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(SINGLE)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0x12))); 
    set_qspi_shakti32(ccr,(CCR_FMODE(CCR_FMODE_INDWR)|CCR_DMODE(QUAD)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(SINGLE)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0x34))); 
    set_qspi_shakti32(ar, address);
    set_qspi_shakti32(dr,value1);
    set_qspi_shakti32(dr,value2);
    set_qspi_shakti32(dr,value3);
    set_qspi_shakti32(dr,value4);
    waitfor(150); 
    wait_for_tcf(status);
    reset_interrupt_flags();
    return wait_for_wip(); // Function which checks if WIP is done, indicating completion of Page Program
}


int flashIdentificationDevice(){
	printf("\tReading the ID register and discovering the Flash Device\n");
	set_qspi_shakti32(dlr,4);
    set_qspi_shakti32(ccr,(CCR_FMODE(CCR_FMODE_INDRD)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0x90)|CCR_DMODE(SINGLE)));
    status = 0; // Useless Variable but still!!!!
    int ret = wait_for_tcf(status);
    int value = get_qspi_shakti(dr);
    reset_interrupt_flags();
	waitfor(1000);
//    if(value == MEM_TYPE_N25Q256_ID){
//    	printf("\tN25Q256 Device Detected \n");
//    	return 0;
//    }
//    else{
//    	printf("\t Device Not Detected - Diagnose %08x\n",value);
//    	return -1;
//    }
}

int flashMemInit(){   //Supposedly a set of routines to check if the memory/interface or whatever is proper
	int ret = flashIdentificationDevice();
	if(ret==-1){
		printf("Flash Mem Init Failed -- Quitting Program, Diagnose");
		return ret;
	}
	else printf("Flash Mem Init Success\n");
    return 0;
	//to fill in code
}

int flashReadStatusRegister(){
//    printf("\tReading the Status bits of the Flash\n");
    set_qspi_shakti32(dlr,4);
    set_qspi_shakti32(ccr,(CCR_FMODE(CCR_FMODE_INDRD)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0x05)|CCR_DMODE(SINGLE)));
 	status = 0;
    int ret = wait_for_tcf(status);
    waitfor(100);
    int value = get_qspi_shakti(dr);
	printf("Read status register value %08x \n", value);
    reset_interrupt_flags();
    if(ret){
        printf("\tRead Status Register Failed\n");
		fail_bit = 1;
        return -1;
        }
    else 
    	return value;
}

int flashReadFlagRegister(){
	return 0;
}


int flashWriteEnable(){
    printf("\tWrite Enable\n");
    set_qspi_shakti32(ccr,(CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0x06)));
    int ret = wait_for_tcf(0); //Indicating the completion of command -- Currently polling
    reset_interrupt_flags();
    return ret; 
}

int flashEnable4ByteAddressingMode(){  //Enable 4-byte addressing Mode and read the status to verify that it has happened correctly

    if(flashWriteEnable()){
        printf("\t Write Enable Failed \n");
        return -1;
    }
    waitfor(100);
    set_qspi_shakti32(ccr,(CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0xB7)));
  //  int status =0; 
    status = 0;
    int ret = wait_for_tcf(status);
    reset_interrupt_flags();
    waitfor(100);
    //Checking Phase
    status = flashReadStatusRegister();
    printf("\t Status Value: %08x\n",status);
    if(status & 1)
        printf("\t 4-Byte Addressing Mode is Enabled\n");
    else
        printf("\t 4-byte Addressing mode not Enabled\n");
}


int flashReadSingleSPI(int dummy_cycles, int read_address, int instruction, int data_words, int adsize){
    set_qspi_shakti32(dlr,data_words);
    set_qspi_shakti32(ccr,(CCR_FMODE(CCR_FMODE_INDRD)|CCR_DMODE(SINGLE)|CCR_DCYC(dummy_cycles)|CCR_ADMODE(SINGLE)|CCR_IMODE(SINGLE)|CCR_ADSIZE(adsize)|CCR_INSTRUCTION(instruction)));
    set_qspi_shakti32(ar,read_address);
    status = 0;
    int ret = wait_for_tcf(status);
    int value = get_qspi_shakti(dr);
    printf("\tThe Read Value: %08x\n",value);
//    printf("\tThe Read Value: %08x\n",value);
    if(ret){
        printf("\t Read Value Failed \n");
        return -1;
    }
    reset_interrupt_flags();
    return value;
}

int flashReadDualSPI(int address, int data_length){
    set_qspi_shakti32(dlr,data_length); //DLR
    set_qspi_shakti32(ccr,(CCR_FMODE(CCR_FMODE_INDRD)|CCR_DMODE(DOUBLE)|CCR_DCYC(3)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(DOUBLE)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0xBC)));
    set_qspi_shakti32(ar,address); 
//    printf("Status : %08x dcr : %08x cr : %08x ccr : %08x dlr: %08x ar: %08x\n",status,*dcr,*cr,*ccr,*dlr,*ar);
    int ret = wait_for_tcf(status);
	int value = get_qspi_shakti(dr);
    printf("\tThe Read Value: %08x\n",value);	
    if(ret){
        printf("\t QSPI: Read Value failed\n");
        return -1;
    }
    //*config_string = get_qspi_shakti(dr);
    //printf("\t Read Data is %08x\n",read_data);
    reset_interrupt_flags();
    return value;
}


int flashReadQuadSPI(int dummy_cycles, int read_address, int instruction, int data_words, int adsize){
     set_qspi_shakti32(dlr,data_words);
     set_qspi_shakti32(ccr,(CCR_FMODE(CCR_FMODE_INDRD)|CCR_DMODE(QUAD)|CCR_DCYC(dummy_cycles)|CCR_ADSIZE(adsize)|CCR_ADMODE(QUAD)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(instruction)));
     set_qspi_shakti32(ar,read_address);
     status = 0;
     int ret = wait_for_tcf(status);
     int value = get_qspi_shakti(dr);
     printf("\tThe Read Value: %08x\n",value);
     if(ret){
         printf("\t Read Value Failed \n");
         return -1; 
     }   
     reset_interrupt_flags();
     return value;
 }

int flashSingleSPIXip(int addr, int* dest_addr){
    set_qspi_shakti32(ccr,(CCR_FMODE(CCR_FMODE_INDRD)|CCR_DMODE(SINGLE)|CCR_DUMMY_CONFIRMATION|CCR_DCYC(7)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(SINGLE)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0x0C)));
    set_qspi_shakti32(dcr,(DCR_FSIZE(27)|DCR_MODE_BYTE(0x00)|DCR_CKMODE(1)));
    set_qspi_shakti32(dlr,0x4);
    set_qspi_shakti32(ar, addr); //Address where the Config_string is situated in the micron nand flash memory
    status=0;
    wait_for_tcf(status);
    waitfor(100); 
    *dest_addr = get_qspi_shakti(dr);
    reset_interrupt_flags(); 
    printf("\t Trying XIP now\n");
    set_qspi_shakti32(ccr, (CCR_FMODE(CCR_FMODE_MMAPD)|CCR_DMODE(SINGLE)|CCR_DUMMY_CONFIRMATION|CCR_DCYC(7)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(SINGLE)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0x0C)));
    set_qspi_shakti32(dcr,(DCR_FSIZE(27)|DCR_MODE_BYTE(0x00)|DCR_CKMODE(1))); 
    waitfor(25);
	return 0;
}

int flashSingleSPIDDRXip(int addr, int* dest_addr){
    set_qspi_shakti32(ccr,(CCR_DDRM|CCR_FMODE(CCR_FMODE_INDRD)|CCR_DMODE(SINGLE)|CCR_DUMMY_CONFIRMATION|CCR_DCYC(15)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(SINGLE)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0x0E)));
    set_qspi_shakti32(dcr,(DCR_FSIZE(27)|DCR_MODE_BYTE(0x00)|DCR_CKMODE(1)));
    set_qspi_shakti32(dlr,0x4);
    set_qspi_shakti32(ar, addr); //Address where the Config_string is situated in the micron nand flash memory
    status=0;
    wait_for_tcf(status);
    waitfor(100); 
    *dest_addr = get_qspi_shakti(dr);
    reset_interrupt_flags(); 
    printf("\t Trying XIP now\n");
    set_qspi_shakti32(ccr, (CCR_DDRM|CCR_FMODE(CCR_FMODE_MMAPD)|CCR_DMODE(SINGLE)|CCR_DUMMY_CONFIRMATION|CCR_DCYC(14)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(SINGLE)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0x0E)));
    set_qspi_shakti32(dcr,(DCR_FSIZE(27)|DCR_MODE_BYTE(0x00)|DCR_CKMODE(1))); 
    waitfor(25);
	return 0;
}

int flashDualSPIXip(int addr, int* dest_addr){
    set_qspi_shakti32(ccr,(CCR_FMODE(CCR_FMODE_INDRD)|CCR_DMODE(DOUBLE)|CCR_DUMMY_CONFIRMATION|CCR_DCYC(3)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(DOUBLE)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0xBC)));
    set_qspi_shakti32(dcr,(DCR_FSIZE(27)|DCR_MODE_BYTE(0xA0)|DCR_CKMODE(1)));
    set_qspi_shakti32(dlr,0x4);
    set_qspi_shakti32(ar, addr); //Address where the Config_string is situated in the micron nand flash memory
    status=0;
    wait_for_tcf(status);
    waitfor(100); 
    *dest_addr = get_qspi_shakti(dr);
    reset_interrupt_flags(); 
    printf("\t Trying XIP now\n");
    set_qspi_shakti32(ccr, (CCR_FMODE(CCR_FMODE_MMAPD)|CCR_DMODE(DOUBLE)|CCR_DUMMY_CONFIRMATION|CCR_DCYC(3)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(DOUBLE)|CCR_IMODE(NDATA)));
    set_qspi_shakti32(dcr,(DCR_FSIZE(27)|DCR_MODE_BYTE(0xA0)|DCR_CKMODE(1))); 
    waitfor(25);
	return 0;
}

int flashDualSPIDDRXip(int addr, int* dest_addr){
    set_qspi_shakti32(ccr,(CCR_DDRM|CCR_FMODE(CCR_FMODE_INDRD)|CCR_DMODE(DOUBLE)|CCR_DUMMY_CONFIRMATION|CCR_DCYC(6)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(DOUBLE)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0xBE)));
    set_qspi_shakti32(dcr,(DCR_FSIZE(27)|DCR_MODE_BYTE(0xA0)|DCR_CKMODE(1)));
    set_qspi_shakti32(dlr,0x4);
    set_qspi_shakti32(ar, addr); //Address where the Config_string is situated in the micron nand flash memory
    status=0;
    wait_for_tcf(status);
    waitfor(100); 
    *dest_addr = get_qspi_shakti(dr);
    reset_interrupt_flags(); 
    printf("\t Trying XIP now\n");
    set_qspi_shakti32(ccr, (CCR_DDRM|CCR_FMODE(CCR_FMODE_MMAPD)|CCR_DMODE(DOUBLE)|CCR_DUMMY_CONFIRMATION|CCR_DCYC(6)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(DOUBLE)|CCR_IMODE(NDATA)));
    set_qspi_shakti32(dcr,(DCR_FSIZE(27)|DCR_MODE_BYTE(0xA0)|DCR_CKMODE(1))); 
    waitfor(25);
	return 0;
}

int flashQuadSPIXip(int addr, int* dest_addr){
    if(flashWriteVolatileConfigReg(0x40404040)){
        printf("\t Volatile Configuration Register not Set -- Diagnose\n");
        return -1;
    }
	status = wait_for_wip();
	printf("\t qspi status register %08x\n",status);
    reset_interrupt_flags();
    set_qspi_shakti32(ccr,(CCR_FMODE(CCR_FMODE_INDRD)|CCR_DMODE(QUAD)|CCR_DUMMY_CONFIRMATION|CCR_DCYC(5)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(QUAD)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0xEC)));
    set_qspi_shakti32(dcr,(DCR_FSIZE(27)|DCR_MODE_BYTE(0xA0)|DCR_CKMODE(1)));
    set_qspi_shakti32(dlr,0x4);
    set_qspi_shakti32(ar, addr); //Address where the Config_string is situated in the micron nand flash memory
    status=0;
    wait_for_tcf(status);
    waitfor(100); 
    *dest_addr = get_qspi_shakti(dr);
    reset_interrupt_flags(); 
    printf("\t Trying XIP now\n");
    set_qspi_shakti32(ccr, (CCR_FMODE(CCR_FMODE_MMAPD)|CCR_DMODE(QUAD)|CCR_DUMMY_CONFIRMATION|CCR_DCYC(5)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(QUAD)|CCR_IMODE(NDATA)));
    set_qspi_shakti32(dcr,(DCR_FSIZE(27)|DCR_MODE_BYTE(0xA0)|DCR_CKMODE(1))); 
    waitfor(25);
	return 0;
}

int flashQuadSPIDDRXip(int addr, int* dest_addr){
    if(flashWriteVolatileConfigReg(0x40404040)){
        printf("\t Volatile Configuration Register not Set -- Diagnose\n");
        return -1;
    }
	status = wait_for_wip();
	printf("\t qspi status register %08x\n",status);
    reset_interrupt_flags();
    set_qspi_shakti32(ccr,(CCR_DDRM|CCR_FMODE(CCR_FMODE_INDRD)|CCR_DMODE(QUAD)|CCR_DUMMY_CONFIRMATION|CCR_DCYC(10)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(QUAD)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0xEE)));
    set_qspi_shakti32(dcr,(DCR_FSIZE(27)|DCR_MODE_BYTE(0xA0)|DCR_CKMODE(1)));
    set_qspi_shakti32(dlr,0x4);
    set_qspi_shakti32(ar, addr); //Address where the Config_string is situated in the micron nand flash memory
    status=0;
    wait_for_tcf(status);
    waitfor(100); 
    *dest_addr = get_qspi_shakti(dr);
    reset_interrupt_flags(); 
    printf("\t Trying XIP now\n");
    set_qspi_shakti32(ccr, (CCR_DDRM|CCR_FMODE(CCR_FMODE_MMAPD)|CCR_DMODE(QUAD)|CCR_DUMMY_CONFIRMATION|CCR_DCYC(10)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(QUAD)|CCR_IMODE(NDATA)));
    set_qspi_shakti32(dcr,(DCR_FSIZE(27)|DCR_MODE_BYTE(0xA0)|DCR_CKMODE(1))); 
    waitfor(25);
	return 0;
}



int flashWriteVolatileConfigReg(int value){
    printf("\t Setting Volatile Configuration Register with the Value: %08x\n",value);
	if(micron_write_enable(status)){
        printf("Panic: Write Enable Command Failed to execute\n");
        return -1;
	}
     reset_interrupt_flags();
//	flashReadStatusRegister();
    set_qspi_shakti32(dlr,DL(1));
    set_qspi_shakti8(dr,value);  //The value to be written into the VECR register to enable XIP. Indicating XIP to operate in Quad Mode
    set_qspi_shakti32(ccr,(CCR_FMODE(CCR_FMODE_INDWR)|CCR_DMODE(SINGLE)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0x01)));
    waitfor(50);
    status=0;
    int ret = wait_for_tcf(status);
    printf("Status : %d dcr : %d cr : %d ccr : %d dlr: %d dr: %d\n",status,*dcr,*cr,*ccr,*dlr,*dr);
    reset_interrupt_flags();
    waitfor(50);  //Giving Micron time to store the data
    return ret;
}

int flash_Write_disable(){
    printf("\tWrite disable\n");
    set_qspi_shakti32(ccr,(CCR_ADSIZE(FOURBYTE)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0x04)));
    int ret = wait_for_tcf(status); //Indicating the completion of command -- Currently polling
    printf("Status : %d dcr : %d cr: %d ccr: %d \n",status,*dcr,*cr,*ccr);
	reset_interrupt_flags();
	return wait_for_wip();
}
	

int eraseSector(int command, int address){
    if(micron_write_enable(status)){
        printf("Panic: Write Enable Command Failed to execute");
        return -1;
    }
    reset_interrupt_flags();
    set_qspi_shakti32(ccr,(CCR_FMODE(CCR_FMODE_INDWR)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(SINGLE)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(command)));
    set_qspi_shakti32(ar,address);
    waitfor(150);
    int ret = wait_for_tcf(status);
    reset_interrupt_flags();
    return wait_for_wip(); //For sector erase maybe estat should be checked
}


int wait_for_wip(){
    int status1;
    do{
        printf("\t Waiting for Wip \n");
        status1 = flashReadStatusRegister(0x05);
        if(check_fail_bit())
            return -1;
        if(status1 & 0x0){
            printf("\tQSPI: Programming Error - Diagnose\n");
            return -1;
        }
        waitfor(20000);
    }while(status1 & 0x01);
    printf("\t QSPI: Page Program/Erase Command Successfully Completed\n");
    return 0;
}


int main()
{
    qspi_init(27,0,3,1,15,1);
 //   uart_init();
    int ar_read,i,j;
    int write_address = 0x0;
    waitfor(100); //Time for Micron to start, maybe?
    if(flashMemInit()) //Because of STARTUPE2 primitive, the run fails for the first time it is programmed since three clock cycles are skipped. Run again
        return -1;  //Didn't work

    int xip_value = 0x40404040;
    
	//Erasing and Writing a bunch of Data
    for(i=0;i<WRITEB;i+=4){
       if(i%ERASE==0){
           printf("\t Beginning of 4KB Sector Erase \n");
        if(eraseSector(0x21, write_address))
            printf("\t Erase Sector Failed -- Diagnose \n");
        else
            printf("\t Erase Completed Successfully \n");
       }
//       pageProgramSingleSPI(write_data[i], write_data[i+1], write_data[i+2], write_data[i+3], write_address); 
       pageProgramSingleSPI(write_data[i], write_data[i+1], write_data[i+2], write_data[i+3], write_address); 
        write_address+=16;
    }
    //Scenarios
    // Fast Read Command (Single SPI Read) is working with 7 Dummy Cycles
    // Quad Fast Read I/O (Quad SPI Read) is working with 9 Dummy Cycles
    waitfor(100);
    flashEnable4ByteAddressingMode();

    //Scenario-1

//    printf("\t SINGLE SPI read with Three Byte with 7 Dummy Cycles\n");

//    ar_read=0;
//    for(i=0;i<WRITEB;++i){
//        flashReadSingleSPI(7,ar_read,0x0B,4,FOURBYTE);
//        waitfor(100);
//        ar_read+=4;
//    }
//	return 0;

    printf("\n\n\n");
	
//	printf("Dual SPI read\n");
//    ar_read=0;
//    for(i=0;i<WRITEB;++i){
//		flashReadDualSPI(ar_read, 4);
//        waitfor(100);
//        ar_read+=4;
//    }
//	return 0;

    printf("\t Quad SPI read, Three Byte Address with 9!! Dummy Cycles 0XEB as the instruction\n");

//    qspi_init(27,0,3,1,15,0); // setting the ck-mode as zero for quad operation
//	status = flashReadStatusRegister();
//	printf("\t qspi status register %08x\n",status);
//    if(flashWriteVolatileConfigReg(xip_value)){
//        printf("\t Volatile Configuration Register not Set -- Diagnose\n");
//        return -1;
//    }
//	status = 0;
//	status = wait_for_wip();
//	printf("\t qspi status register %08x\n",status);
//    reset_interrupt_flags();
//    ar_read=0;
//    for(i=0;i<WRITEB;++i){
//        flashReadQuadSPI(5,ar_read,0xEB,4,FOURBYTE);
//        waitfor(100);
//        ar_read+=4;
//    }
//    return 0;
    printf("\n\n\n");

    //XIP Mode
//    int xip_value = 0x40404040;
//    if(flashWriteVolatileConfigReg(xip_value)){
//        printf("\t Volatile Configuration Register not Set -- Diagnose\n");
//        return -1;
//    }
    printf("\t Quad I/O Mode with Dummy Confirmation bit to enable XIP\n");

	
//    set_qspi_shakti32(ccr,(CCR_DDRM|CCR_FMODE(CCR_FMODE_INDRD)|CCR_DMODE(SINGLE)|CCR_DUMMY_CONFIRMATION|CCR_DCYC(15)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(SINGLE)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0x0E)));
//    set_qspi_shakti32(dcr,(DCR_FSIZE(27)|DCR_MODE_BYTE(0x00)|DCR_CKMODE(1)));
//    set_qspi_shakti32(dlr,0x4);
//    set_qspi_shakti32(ar,0x000000); //Address where the Config_string is situated in the micron nand flash memory
//    status=0;
//    wait_for_tcf(status);
//    waitfor(100); 
    int *config_string = (const int*)0x80010000;
    int* read_data = (int*)(0x90000004); //Read data register after the first element is read through QUAD mode
//    flashSingleSPIXip(0x000000, config_string);
//	flashSingleSPIDDRXip(0x000000, config_string);
//    flashDualSPIXip(0x000000, config_string);
//	flashDualSPIDDRXip(0x000000, config_string);
//	flashQuadSPIXip(0x000000, config_string);
	flashQuadSPIDDRXip(0x000000, config_string);
//    *config_string = get_qspi_shakti(dr);
//    printf("\tRead data is : %08x\n",*config_string);
      config_string++;
//    reset_interrupt_flags(); 
//    printf("\t Trying XIP now\n");
//    set_qspi_shakti32(ccr, (CCR_DDRM|CCR_FMODE(CCR_FMODE_MMAPD)|CCR_DMODE(SINGLE)|CCR_DUMMY_CONFIRMATION|CCR_DCYC(14)|CCR_ADSIZE(FOURBYTE)|CCR_ADMODE(SINGLE)|CCR_IMODE(SINGLE)|CCR_INSTRUCTION(0x0E)));
//    set_qspi_shakti32(dcr,(DCR_FSIZE(27)|DCR_MODE_BYTE(0x00)|DCR_CKMODE(1))); 
//    waitfor(25);
    int dum_data;

    for(i=0;i<WRITEB;++i) {
         dum_data = get_qspi_shakti(read_data);
         waitfor(100);
         *config_string = dum_data;
         config_string++;
         read_data++;
         waitfor(100);
         reset_interrupt_flags();
         waitfor(10);
    }
    config_string = (const int*)(0x80010000);
    for(i=0;i<WRITEB;++i){
        printf("Data: %08x Address: %08x \n",*config_string, config_string);
        config_string++;
    }
//    uart_finish();
	return 0;
}
