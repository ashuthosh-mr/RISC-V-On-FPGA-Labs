`define COLUMN_WIDTH 12             //Corresponds to No. of bytes needed in a page. Eg : 2^12 = 4kB  
`define PAGE_WIDTH 9                //Corresponds to No. of pages needed in a block. Eg : 2^6 = 64 Pages 
`define BLOCK_WIDTH 11              //Corresponds to No. of blocks needed in a PLANE(not in a LUN). Eg : 2^10 = 1024 blocks per plane 
`define PLANE_WIDTH 1               //Corresponds to No. of planes needed in a LUN. This s FIXED to 2^1 = 2.
`define LUN_WIDTH 1                 //Corresponds to No. of LUNS needed in a target chip. This s FIXED to 2^1 = 2.
`define TOTAL_PLANE 4               //#LUNS * #planes. Fixed
`define VALID_COL_ADDR 4320 //16383 //edited on 10thmay        //(2^COLUMN_WIDTH-1)+SPARE_BYTES_AREA(Say 64B)
`define WDC 64                      //NVMe - NFC Bus width
`define TOTAL_CHIPS 2               //Total chips per NFC
`define MAX_ROW_BITS 22             // This changes with different Flash Classes. Need to refer to nand_parameters.vh for this value

`define VALID_SPARE_AREA 224 //1215  //edited on 10thma  // Size of spare area in Bytes - 1, (Say 0 to 63B total of 64B) in whole page.
`define SECTOR_SPARE_SIZE 28 //38 //edited on 10thmay // Size of spare area for each sector.  (VALID_SPARE_AREA+1)*512/(2^COLUMN_WIDTH)

`define FIFO_ROWS 2047              //Page_size/WDC -1
`define PAGE_LENGTH 511
`define LBPR 3                      //Log Bytes Per Row. logbase2(WDC/8) .

//`define PADDR	64
//`define Reg_width 64
//`define USERSPACE 12

typedef 1 Num_Slaves ;
typedef 1 Num_Masters ;
typedef 1 Slave_num;

//Based on above value comment out suitable things below.
//`define COLUMN_WIDTH_LT_8
//`define COLUMN_WIDTH_E_8
`define COLUMN_WIDTH_GT_8

`define PAGE_WIDTH_LT_8
//`define PAGE_WIDTH_E_8
//`define PAGE_WIDTH_GT_8

`define PAGE_PLANE_WIDTH_LT_8
//`define PAGE_PLANE_WIDTH_E_8
//`define PAGE_PLANE_WIDTH_GT_8

`define PAGE_PLANE_BLOCK_WIDTH_LTE_8
//`define PAGE_PLANE_BLOCK_WIDTH_GT_8

`define PAGE_PLANE_BLOCK_WIDTH_LTE_16
//`define PAGE_PLANE_BLOCK_WIDTH_GT_16

`define PAGE_PLANE_BLOCK_LUN_WIDTH_LTE_8
//`define PAGE_PLANE_BLOCK_LUN_WIDTH_GT_8

`define PAGE_PLANE_BLOCK_LUN_WIDTH_LTE_16
//`define PAGE_PLANE_BLOCK_LUN_WIDTH_GT_16


`define TIMING_MODE 'h11

//Choose CLK_PERIOD >= tWP*2
`define VCC_DELAY 5002                 // ceil(tVCC/CLK_PERIOD). For simulation setting this value small but valid >10000
`define TWP_MODE0 17                  // ceil(tWP mode0 / CLK_PERIOD) >50
`define TWB_MODE0 33                  // ceil(tWB mode0 / CLK_PERIOD) <200
`define TADL_MODE0 34                  // ceil(tADL mode0 / CLK_PERIOD) >200
`define TWB_COUNT 16                // ceil(tWB/CLK_PERIOD) <100
`define TWB_COUNT_RD 16             // for sync read 
`define TWHR_COUNT 14                 // ceil(tWHR/CLK_PERIOD) >60
`define TWHR_RCOUNT 14                 // ceil(tWHR/CLK_PERIOD) >60
`define TRHW_COUNT 16                 // ceil(tRHW/CLK_PERIOD) >100
`define TCCS_COUNT 51               // ceil(tCCS/CLK_PERIOD) >300
`define TADL_COUNT 14                 // ceil(tADL/CLK_PERIOD) >70
`define TCALS_COUNT 1 
`define TWB_TR_COUNT 10 
/*`define TWH_COUNT 1		     // ceil(tWH/CLK_PERIOD ) >7
`define TWP_COUNT 1		     // ceil(tWH/CLK_PERIOD ) >10    */



typedef enum {
	IDLE ,
	PROGRAM_PAGE ,
	PROGRAM_PAGE_CACHE ,
	PROGRAM_PAGE_MULTI_PLANE 
} Program_states deriving( Bits, Eq ) ;

typedef enum {
	IDLE ,
	READ_PAGE ,
	READ_PAGE_MULTI_PLANE 
} Read_states deriving( Bits, Eq ) ;


typedef enum {
	ASK_STATUS1 ,
	ASK_STATUS2 ,
	P_ADDR_S ,
	B_ADDR_S ,
	L_ADDR_S ,
	ENABLE_S_READ1 ,
	ENABLE_S_READ2 ,
	WAIT1 ,
	WAIT2 ,
	WAIT_RE_WE1 ,
	WAIT_RE_WE2 ,
    WAIT_NEW_1,
    WAIT_NEW_3,
	WAIT_STATUS_PLANE1 ,
	WAIT_D1 ,
	WAIT_D2 ,
	WAIT_D3 ,
	WAIT_D4 ,
	WAIT_D5 ,
	WAIT_D6 ,
	WAIT_D7 ,
	WAIT_TWB ,
    WAIT_FOR_tRHW,
	READ_STATUS1 ,
	READ_STATUS2 ,
	START_PROGRAM ,
	C_ADDR ,
	C_ADDR1 ,
	P_ADDR ,
	B_ADDR ,
	L_ADDR ,
    ECC_REQ,
	START_DATA ,
	END_COMMAND ,
	DUMMY
} Program_operation_state deriving(Bits, Eq, FShow) ;


typedef enum {
	ASK_STATUS1 ,
	P_ADDR_S ,
	B_ADDR_S ,
	L_ADDR_S ,
	ENABLE_S_READ1 ,
	WAIT1 ,
    WAIT_FOR_tRHW,
	WAIT3 ,
	WAIT4 ,
	WAIT_RE_WE1 ,
	WAIT_RE_WE3 ,
	WAIT_STATUS_PLANE1 ,
	WAIT_D1 ,
	WAIT_D2 ,
	WAIT_D3 ,
	WAIT_D4 ,
	WAIT_D5 ,
	WAIT_D6 ,
	WAIT_D7 ,
	WAIT_TWB ,
	WAIT_TWB1 ,
	WAIT_TCCS1 ,
	WAIT_TCCS2 ,
	READ_STATUS1 ,
	C_ADDR ,
	C_ADDR1 ,
	C_ADDR_1 ,
	C_ADDR_2 ,
	P_ADDR ,
	P_ADDR_1 ,
	B_ADDR ,
	B_ADDR_1 ,
	L_ADDR ,
	L_ADDR_1 ,
	START_READ ,
	START_READ_C ,
	CONT_READ ,
	END_COMMAND ,
    END_COMMAND_1,
	SELECT_C_R ,
	DUMMY,
    WAIT_NEW_1,
    WAIT_NEW_2,
    WAIT_NEW_3
} Read_operation_state deriving(Bits, Eq, FShow) ;


typedef enum {
	ASK_STATUS1 ,
    WAIT_D1,
    WAIT_NEW,
    P_ADDR_S,
    B_ADDR_S,
    L_ADDR_S,
    WAIT_RE_WE1,
    ENABLE_S_READ1,
    WAIT1,
    WAIT_FOR_tRHW,
    READ_STATUS1,
    START_ERASE,
    WAIT_D3,
    WAIT_NEW_1,
    P_ADDR,
    B_ADDR,
    L_ADDR,
    WAIT_D4,
    END_COMMAND,
    WAIT_TWB1,
    WAIT_TWB2,
    WAIT_D7,
    WAIT_D5,
    WAIT_TWB,
    WAIT_STATUS_PLANE1,
    FINISH_STATUS,
    WAIT_D6,
    WAIT_NEW_3,
    P_ADDR_S1,
    B_ADDR_S1,
    L_ADDR_S1,
    WAIT_RE_WE2,
    ENABLE_S_READ3,
    WAIT3,
    READ_STATUS3,
    WAIT_FOR_tRHW1
} Erase_operation_state deriving(Bits, Eq, FShow) ;



typedef enum {
    GET_ADDR,
    START_READ,
    WAIT_D1,
    WAIT_NEW,
    C_ADDR,
    C_ADDR1,
    P_ADDR,
    B_ADDR,
    L_ADDR,
    WAIT_D2,
    END_COMMAND,
    WAIT_D3,
    WAIT_TWB,
    WAIT_D7,
    WAIT_TWB1,
    WAIT_STATUS_PLANE1,
    ASK_STATUS1,
    WAIT_D8,
    WAIT_NEW_1,
    P_ADDR_S1,
    B_ADDR_S1,
    L_ADDR_S1,
    WAIT_RE_WE1,
    ENABLE_S_READ1,
    WAIT1,
    READ_STATUS1,
    WAIT_FOR_tRHW,
    START_READ_C,
    WAIT_TCCS2,
    WAIT3,
    CONT_READ,
    CONT_READ1,
    WAIT_RE_WE3,
    SELECT_C_R,
    WAIT_D4,
    WAIT_NEW_2,
    C_ADDR_1,
    C_ADDR_2,
    P_ADDR_1,
    B_ADDR_1,
    L_ADDR_1,
    WAIT_D5,
    END_COMMAND_1,
    WAIT_TCCS1,
    WAIT_RE_WE5,
    ASK_STATUS2,
    WAIT_D6,
    WAIT_NEW_3,
    P_ADDR_S,
    B_ADDR_S,
    L_ADDR_S,
    WAIT_RE_WE4,
    ENABLE_S_READ2,
    WAIT2,
    READ_STATUS2,
    DUMMY
} BBM_operation_state deriving(Bits, Eq, FShow);

typedef enum{
    IDLE,
    START_PROGRAM,
    WAIT_D1,
    WAIT_D2,
    C_ADDR,
    WAIT_D4,
    WAIT_D3,
    START_DATA,
    WAIT_D6,
    WAIT_D7,
    WAIT_TWB,
    WAIT_D5,
    DUMMY
} Feature_operation_state deriving(Bits, Eq, FShow);




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////                                            Function to map address to physical locations on chip                                                /////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Here need to map to physical address in target for high band width	*/
function Bit#(64) fn_map_address (Bit#(64) addr_from_nvm);
Bit#(TSub#(`BLOCK_WIDTH,`LUN_WIDTH)) partial_addr1 = addr_from_nvm[`COLUMN_WIDTH+`PAGE_WIDTH+`BLOCK_WIDTH+`PLANE_WIDTH:`COLUMN_WIDTH+`PAGE_WIDTH+`PLANE_WIDTH+`LUN_WIDTH+1];
Bit#(1) partial_addr2 = (addr_from_nvm[`COLUMN_WIDTH]^addr_from_nvm[`COLUMN_WIDTH+`PAGE_WIDTH+`LUN_WIDTH+1]);
Bit#(`PAGE_WIDTH) partial_addr3 = addr_from_nvm[`COLUMN_WIDTH+`PAGE_WIDTH+`LUN_WIDTH:`COLUMN_WIDTH+`LUN_WIDTH+1];
Bit#(`COLUMN_WIDTH) partial_addr4 = addr_from_nvm[`COLUMN_WIDTH-1:0];
//return({'h0,addr_from_nvm[`COLUMN_WIDTH+`LUN_WIDTH],partial_addr1,partial_addr2,addr_from_nvm[`COLUMN_WIDTH],partial_addr3,partial_addr4});
if((`PAGE_WIDTH+`BLOCK_WIDTH+`PLANE_WIDTH) == (`MAX_ROW_BITS-1)) //I dont need to add zeros b/t LUN bit and BA bits
	return({'h0,addr_from_nvm[`COLUMN_WIDTH+`LUN_WIDTH],partial_addr1,partial_addr2,addr_from_nvm[`COLUMN_WIDTH],partial_addr3,partial_addr4});
else //Need to add zeros b/t LUN bit and block addr bit
begin
	Bit#(TSub#(TSub#(`MAX_ROW_BITS,1),TAdd#(`PAGE_WIDTH,TAdd#(`BLOCK_WIDTH,`PLANE_WIDTH)))) partial_addr7 = 'h0;
	return({'h0,addr_from_nvm[`COLUMN_WIDTH+`LUN_WIDTH],partial_addr7,partial_addr1,partial_addr2,addr_from_nvm[`COLUMN_WIDTH],partial_addr3,partial_addr4});
end
endfunction

//Funtion to increment page address by 1.
function Bit#(64) fn_get_next_nvm_addr (Bit#(64) addr_from_nvm);
Bit#(TAdd#(TAdd#(TAdd#(`PAGE_WIDTH,`BLOCK_WIDTH),`PLANE_WIDTH),`LUN_WIDTH)) partial_addr5 = addr_from_nvm[`COLUMN_WIDTH+`PAGE_WIDTH+`BLOCK_WIDTH+`PLANE_WIDTH+`LUN_WIDTH-1:`COLUMN_WIDTH]+'h1;
Bit#(`COLUMN_WIDTH) partial_addr6 = addr_from_nvm[`COLUMN_WIDTH-1:0] ;
return({'h0,partial_addr5,partial_addr6}); 
endfunction



interface NandFlashInterface   ;
	method Action _request_data(Bit#(64) _address, Bit#(11) _length);
	method Action _request_erase(Bit#(64) _address); //To erase a block of data
	method ActionValue#(Bit#(`WDC)) _get_data_();
	method Action _write_addr(Bit#(64) _address, Bit#(11) _length);
    method Action _write_data(Bit#(`WDC) _data);
	method Action _enable(bit _nand_ce_l);
	method Action _query_bad_block(Bit#(64) _address);
	method bit interrupt_();
	method bit busy_();
	method bit write_success_();
	method bit write_fail_();
	method bit erase_success_();
	method bit erase_fail_();
    method Bit#(1) _read_valid();
endinterface

interface ONFi_cfg_reg_interface;
    method Action _timing_mode(Bit#(8) data);
endinterface


// Methods for NVM-NFC interface definitions
//function NandFlashInterface fn_nvm_nfc_interface (Wire#(Bit#(64)) wr_address_from_nvm,Wire#(Bit#(`WDC)) wr_data_from_nvm,Wire#(Bit#(11)) wr_w_length,Wire#(Bit#(11)) wr_r_length, Reg#(Maybe#(Bit#(`WDC))) rg_data_to_nvm,Wire#(Bit#(1)) wr_nand_ce_n,Wire#(Bit#(1)) wr_nand_we_n, Wire#(Bit#(1)) wr_nand_re_n,Reg#(Bit#(1)) rg_interrupt,Reg#(Bit#(1)) rg_ready_busy_n, Reg#(Bit#(1)) wr_nand_erase, Reg#(Bit#(1)) rg_write_success, Reg#(Bit#(1)) rg_write_fail, Reg#(Bit#(1)) rg_erase_success, Reg#(Bit#(1)) rg_erase_fail, Wire#(Bit#(1)) wr_nand_bbm_n);
//	
//	return (interface NandFlashInterface ;
//                      
//		      method Action _request_data(_address,_length) ;
//                  wr_nand_re_n	  <= 1'b0;
//                  wr_nand_ce_n <= 1'b0;
//                  wr_nand_we_n	  <= 1'b1;
//                  wr_address_from_nvm <= pack(_address);
//                  wr_r_length         <= pack(_length);
//       	      endmethod: _request_data
//		      
//		      method Action _request_erase(_address) ;
//                  wr_nand_erase	  <= 1'b1;
//                  wr_nand_ce_n <= 1'b0;
//                  wr_address_from_nvm <= pack(_address);
//              endmethod: _request_erase
//
//              method Maybe#(Bit#(`WDC)) _get_data_() ;
//                  return rg_data_to_nvm ;
//              endmethod: _get_data_
//
//              method Action _write_addr(_address, _length) ;
//                  wr_address_from_nvm <= pack(_address) ;
//                  wr_w_length         <= pack(_length) ;
//              endmethod: _write_addr
//
//              method Action _write_data(_data);
//                  wr_data_from_nvm    <= _data;
//                  wr_nand_ce_n <= 1'b0;
//                  wr_nand_we_n	  <= 1'b0;
//              endmethod
//
//              method Action _enable(bit _nand_ce_l);
////                wr_nand_ce_n        <= _nand_ce_l;
//              endmethod: _enable
//
//              method bit interrupt_() ;
//                  return rg_interrupt;           //Interrupt when read data is ready 
//              endmethod: interrupt_
//
//              method bit busy_() ;              	              
//                  return (~rg_ready_busy_n);     //Return busy state of the NAND Controller
//              endmethod: busy_
//		      
//		      method bit write_success_() ;              	              
//  	              return (rg_write_success);     
//       	      endmethod: write_success_
//		      
//		      method bit write_fail_() ;              	              
//                  return (rg_write_fail);     
//              endmethod: write_fail_
//		      
//		      method bit erase_success_() ;              	              
//                  return (rg_erase_success);     
//              endmethod: erase_success_
//		      
//		      method bit erase_fail_() ;              	              
//                  return (rg_erase_fail);     
//              endmethod: erase_fail_
//		      
//		      method Action _query_bad_block(_address) ;
//                  wr_nand_bbm_n	  <= 1'b0     ;
//                  wr_address_from_nvm <= pack(_address) ;
//              endmethod: _query_bad_block
//		
//          endinterface );
//endfunction


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		NAND FLASH CONTROLLER - ONFi INTERFACE
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

interface Onfi_Interface_top   ;
 
	method bit onfi_ce0_n_ () ;
	method bit onfi_ce1_n_ () ;
	method bit onfi_we_n_ ();
	method bit onfi_re_n_ () ;
	method bit onfi_wp_n_ () ;
	method bit onfi_cle_ () ;
	method bit onfi_ale_ () ;
	method Action _ready_busy0_n_m ( bit _ready_busy0_l ) ;
	method Action _ready_busy1_n_m ( bit _ready_busy1_l ) ;
    method Bool dqs_out;
    method Bit#(1) dqs_enable;
    method Action dqs_in(Bit#(1) dqin);
    method Bool dqs_c_out;
    method Bit#(1) dqs_c_enable;
    method Action dqs_c_in(Bit#(1) dqcin);
    method Bit#(8) data0_out;
    method Bit#(1) data0_enable;
    method Action data0_data_in(Bit#(8) data0_in);
    method Bit#(8) data1_out;
	method Bit#(1) data1_enable;
    method Action data1_data_in(Bit#(8) data1_in);
    method Bool dqs2_out;
    method Bit#(1) dqs2_enable;
    method Action dqs2_in(Bit#(1) dq2_in);
    method Bool dqs2_c_out;
    method Bit#(1) dqs2_c_enable;
    method Action dqs2_c_in(Bit#(1) dq2_cin);

endinterface



interface ONFiInterface   ;
 
	method bit onfi_ce0_n_ () ;
	method bit onfi_ce1_n_ () ;
	method bit onfi_sync_we_n_ ();
    method bit onfi_async_we_n_ ();
    method bit timing_set_();
	method bit onfi_re_n_ () ;
	method bit onfi_wp_n_ () ;
	method bit onfi_cle_ () ;
	method bit onfi_ale_ () ;
	method Action _ready_busy0_n_m ( bit _ready_busy0_l ) ;
	method Action _ready_busy1_n_m ( bit _ready_busy1_l ) ;
    method Bool dqs_out;
    method Bit#(1) dqs_enable;
    method Action dqs_in(Bit#(1) dqin);
    method Bool dqs_c_out;
    method Bit#(1) dqs_c_enable;
    method Action dqs_c_in(Bit#(1) dqcin);
    method Bit#(8) data0_out;
    method Bit#(1) data0_enable;
    method Action data0_data_in(Bit#(8) data0_in);
    method Bit#(8) data1_out;
	method Bit#(1) data1_enable;
    method Action data1_data_in(Bit#(8) data1_in);
    method Bool dqs2_out;
    method Bit#(1) dqs2_enable;
    method Action dqs2_in(Bit#(1) dq2_in);
    method Bool dqs2_c_out;
    method Bit#(1) dqs2_c_enable;
    method Action dqs2_c_in(Bit#(1) dq2_cin);

endinterface

//Methods for NFC-ONFI interface definitions
function ONFiInterface fn_nfc_onfi_interface (Reg#(bit) rg_onfi_ce0_n,Reg#(bit) rg_onfi_ce1_n,Reg#(bit) rg_onfi_cle,Reg#(bit) rg_onfi_ale,
Reg#(bit) rg_onfi_sync_we_n, Reg#(bit) rg_onfi_async_we_n, Reg#(bit) rg_timing_set, Reg#(bit) rg_onfi_re_n,Reg#(bit) rg_onfi_wp_n,
Wire#(bit) wr_ready_busy0_n,Wire#(bit) wr_ready_busy1_n,Reg#(Bool) rg_dqs_out, Wire#(Bit#(1)) rg_dqs_enable, Reg#(Bit#(1)) rg_dqs_in,
Reg#(Bool) rg_dqs_c_out, Reg#(Bit#(1)) rg_dqs_c_enable, Reg#(Bit#(1)) rg_dqs_c_in,Reg#(Bool) rg_dqs2_out,
Wire#(Bit#(1)) rg_dqs2_enable, Reg#(Bit#(1)) rg_dqs2_in, Reg#(Bool) rg_dqs2_c_out, Reg#(Bit#(1)) rg_dqs2_c_enable, Reg#(Bit#(1)) rg_dqs2_c_in, Reg#(Bit#(8)) rg_data0_out, Reg#(Bit#(1)) rg_data0_enable, Reg#(Bit#(8)) rg_data0_in,
Reg#(Bit#(8)) rg_data1_out, Reg#(Bit#(1)) rg_data1_enable, Reg#(Bit#(8)) rg_data1_in);
	
	return (interface ONFiInterface ;

               method Bool dqs_out;
                   return rg_dqs_out;
               endmethod

               method Bit#(1) dqs_enable;
                   return rg_dqs_enable;
               endmethod

               method Action dqs_in(Bit#(1) dqin);
                   rg_dqs_in <= dqin;
               endmethod                   

               method Bool dqs_c_out;
                   return rg_dqs_c_out;
               endmethod

               method Bit#(1) dqs_c_enable;
                   return rg_dqs_c_enable;
               endmethod

               method Action dqs_c_in(Bit#(1) dqcin);
                   rg_dqs_c_in <= dqcin;
               endmethod

               method Bool dqs2_out;
                   return rg_dqs2_out;
               endmethod

               method Bit#(1) dqs2_enable;
                   return rg_dqs2_enable;
               endmethod

               method Action dqs2_in(Bit#(1) dq2_in);
                   rg_dqs2_in <= dq2_in;
               endmethod                   

               method Bool dqs2_c_out;
                   return rg_dqs2_c_out;
               endmethod

               method Bit#(1) dqs2_c_enable;
                   return rg_dqs2_c_enable;
               endmethod

               method Action dqs2_c_in(Bit#(1) dq2_cin);
                   rg_dqs2_c_in <= dq2_cin;
               endmethod

               method Bit#(8) data0_out;
                   return rg_data0_out;
               endmethod

               method Bit#(1) data0_enable;
                   return rg_data0_enable;
               endmethod

               method Action data0_data_in(Bit#(8) data0_in);
                   rg_data0_in <= data0_in;
               endmethod

               method Bit#(8) data1_out;
                   return rg_data1_out;
               endmethod

		       method Bit#(1) data1_enable;
                   return rg_data1_enable;
               endmethod

               method Action data1_data_in(Bit#(8) data1_in);
                   rg_data1_in <= data1_in;
               endmethod

		       method bit onfi_ce0_n_ () ;
                       return rg_onfi_ce0_n ;
               endmethod: onfi_ce0_n_

		       method bit onfi_ce1_n_ () ;
	                   return rg_onfi_ce1_n ;
	           endmethod: onfi_ce1_n_		      

	           method bit onfi_sync_we_n_ () ;
	                   return rg_onfi_sync_we_n ;
	           endmethod: onfi_sync_we_n_	

	           method bit onfi_async_we_n_ () ;
	                   return rg_onfi_async_we_n ;
	           endmethod: onfi_async_we_n_

               method bit timing_set_();
                       return rg_timing_set;
               endmethod

	           method bit onfi_re_n_ () ;
	                   return rg_onfi_re_n ;
	           endmethod: onfi_re_n_	

	           method bit onfi_wp_n_ () ;
	                   return rg_onfi_wp_n ;
	           endmethod: onfi_wp_n_	

	           method bit onfi_cle_ () ;
	                   return rg_onfi_cle ;
	           endmethod: onfi_cle_	

	           method bit onfi_ale_ () ;
	                   return rg_onfi_ale ;
	           endmethod: onfi_ale_
	
	           method Action _ready_busy0_n_m ( _ready_busy_n ) ;
	                   wr_ready_busy0_n <= _ready_busy_n ;
	           endmethod: _ready_busy0_n_m
		      
		       method Action _ready_busy1_n_m ( _ready_busy_n ) ;
	                       wr_ready_busy1_n <= _ready_busy_n ;
               endmethod: _ready_busy1_n_m
		
		endinterface );
endfunction


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		NVM - ECC ENCODER INTERFACE
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
interface NFC_ECCencoderInterface   ;
 
	method Action _data_from_ecc_encoder ( Bit#(8) _data_from_ecc_encoder ) ;
	method Bit#(8) data_to_ecc_encoder ( ) ;
	method bit ecc_encoder_ce () ;
	method bit ecc_encoder_start () ;

endinterface

//Methods for NFC-ECC ENCODER definitions
function NFC_ECCencoderInterface fn_nfc_ecc_encoder_interface (Reg#(bit) rg_ecc_enc_ce,Reg#(bit) rg_ecc_enc_start,Reg#(Bit#(8)) rg_data_to_ecc_enc, Wire#(Bit#(8)) wr_data_from_ecc_enc) ;
	
	return (interface NFC_ECCencoderInterface ;
	               		       
	               method Action _data_from_ecc_encoder ( data_from_ecc_encoder ) ;
	                       wr_data_from_ecc_enc <= data_from_ecc_encoder ;
	               endmethod: _data_from_ecc_encoder
	
	               method Bit#(8) data_to_ecc_encoder ( ) ;
	                       return rg_data_to_ecc_enc ;
	               endmethod: data_to_ecc_encoder
	
	               method bit ecc_encoder_ce () ;
	                       return rg_ecc_enc_ce ;
	               endmethod: ecc_encoder_ce
		      
		       method bit ecc_encoder_start () ;
	                       return rg_ecc_enc_start ;
	               endmethod: ecc_encoder_start
		       	
		endinterface );
endfunction

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		NVM - ECC DECODER INTERFACE
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
interface NFC_ECCdecoderInterface   ;
    method Bit#(8) data_to_ecc_dec ;
    method bit syn_start_ecc_dec;
    method Action osyn_ready (bit sync_ready);
    method Action oerr_out (Bit#(8) err_out);
    method Action ofirst_out (bit first_out);
endinterface

//Methods for NFC-ECC DECODER definitions
function NFC_ECCdecoderInterface fn_nfc_ecc_decoder_interface (Reg#(Bit#(8)) rg_data_in, 
    Reg#(bit) rg_sync_start, Wire#(bit) wr_osyn_ready, Wire#(Bit#(8)) wr_oerr_out, 
    Wire#(Bit#(1)) wr_ofirst_out);

	return (interface NFC_ECCdecoderInterface ;
                   
                   method Bit#(8) data_to_ecc_dec ;
                       return rg_data_in;
                   endmethod
                   method bit syn_start_ecc_dec;
                       return rg_sync_start;
                   endmethod
                   method Action osyn_ready (bit sync_ready);
                        wr_osyn_ready <= sync_ready;
                   endmethod
                   method Action oerr_out (Bit#(8) err_out);
                        wr_oerr_out <= err_out;
                   endmethod
                   method Action ofirst_out (bit first_out);
                        wr_ofirst_out <= first_out;
                   endmethod       
		endinterface );

endfunction
