package debug_types;

  import GetPut :: *;
  import AXI4_Types::*;
  import AXI4_Lite_Types::*;
  import Connectable::*;
  // Limit What is Exported !

  // Constants
	typedef enum {    Abst_NoError = 3'b000        , Abst_Busy = 3'b001,
			          		Abst_NotSupported = 3'b010   , Abst_Exception = 3'b011,
			          		Abst_WrongState = 3'b100     , Abst_Bus       = 3'b101,
                    Abst_Other = 3'b111 }	Abst_ErrorTypes deriving(Bits,Eq,FShow);

  typedef enum {  SbNoError = 3'b000        , SbTimeOut = 3'b001,
                    SbBadAddress = 3'b010     , SbAlign = 3'b011,
                    SbSize = 3'b100           ,
                    SbOther = 3'b111 } SbErrorTypes deriving(Bits,Eq,FShow);

  // For ABITS == 7
  typedef 7'h04 ABSTRACTDATASTART;
  typedef 7'h0f ABSTRACTDATAEND;
  typedef 7'h10 DMCONTROL;
  typedef 7'h11 DMSTATUS;
  typedef 7'h12 HARTINFO;
  typedef 7'h13 HALTSUM1;
  typedef 7'h14 HAWINDOWSEL;
  typedef 7'h15 HAWINDOW;
  typedef 7'h16 ABSTRACTCTS;
  typedef 7'h17 COMMAND;
  typedef 7'h18 ABSTRACTAUTO;
  typedef 7'h19 CONFIGSTRINGADDR0;
  typedef 7'h1a CONFIGSTRINGADDR1;
  typedef 7'h1b CONFIGSTRINGADDR2;
  typedef 7'h1c CONFIGSTRINGADDR3;
  typedef 7'h1d NEXTDM;
  typedef 7'h20 PBSTART;
  typedef 7'h2f PBEND;
  typedef 7'h30 AUTHDATA;
  typedef 7'h34 HALTSUM2;
  typedef 7'h35 HALTSUM3;
  typedef 7'h36 SBADDRESS3;
  typedef 7'h38 SBCS;
  typedef 7'h39 SBADDRESS0;
  typedef 7'h3a SBADDRESS1;
  typedef 7'h3a SBADDRESS2;
  typedef 7'h3c SBDATA0;
  typedef 7'h3d SBDATA1;
  typedef 7'h3e SBDATA2;
  typedef 7'h3f SBDATA3;
  typedef 7'h40 HALTSUM0;

  // Abst Reg Map
  typedef 14  AbstractAddrWidth;  // Limited Extended abstract reg space

  typedef 14'h0000 Abst_reg_address_CSR0;
  typedef 14'h1000 Abst_reg_address_GPR0;                          // ref. AbstractAddrWidth for 14.
  typedef 14'h1020 Abst_reg_address_FPR0;

  // ---      Implementation Defined Parameters     --- //

  // ConfigString Pointer
  typedef 0 D_configstrptr0;
  typedef 0 D_configstrptr1;
  typedef 0 D_configstrptr2;
  typedef 0 D_configstrptr3;

  `ifdef RV64
    typedef 64  DXLEN;
  `else 
    typedef 32  DXLEN;
  `endif
  //typedef 64  DXLEN;
  typedef 32  DPADDR;
  typedef 1   HartCount;
  typedef 1   AxiID;

  typedef struct {
    Bool        read_write;
    Bit#(14)    address;
    Bit#(DXLEN) writedata;
  `ifdef spfpu
    Bool        rftype;   // false:irf, true:frf
  `endif } AbstractRegOp deriving(Bits, Eq, FShow);


  // Helpful Macros
  `define FIVO(x) fromInteger(valueOf(x))

  // Interface
    //Interface between Debug Module and DTM (eg. JtagDTM)
	interface Ifc_DM_DTM;
    interface Put#(Bit#(41)) putCommand;// 7 (ABITS) + 32 + 2
    interface Get#(Bit#(34)) getResponse;
    interface Reset dmactive_reset;
  endinterface

  
  interface Debug_Hart_Ifc;
    method ActionValue#(AbstractRegOp) abstractOperation;
    method Action  abstractReadResponse(Bit#(DXLEN) abstractResponse);  
    (*always_enabled,always_ready*)
    method Bit#(1) haltRequest();
    (*always_enabled,always_ready*)
    method Bit#(1) resumeRequest();
    (*always_enabled,always_ready*)
    method Bit#(1) hart_reset();                               // Signal TO Reset HART -Active HIGH
    (*always_enabled,always_ready*)
    method Action  set_have_reset(Bit#(1) have_reset);
    (*always_enabled,always_ready*)
    method Action  set_halted(Bit#(1) halted);
    (*always_enabled,always_ready*)
    method Action  set_unavailable(Bit#(1) unavailable);  
    (*always_enabled,always_ready*)
    method Bit#(1) dm_active;
    // method Bit#(5) Hartsel; Information to abstract bus to reduce wires fo the multi hart case 
  endinterface
    
	// Interface between Debug Module and SOC
  interface Ifc_riscvDebug013;
    interface Ifc_DM_DTM dtm;
    interface Debug_Hart_Ifc hart;
  `ifdef CORE_AXI4
    interface AXI4_Master_IFC#(DPADDR, DXLEN, 0 ) debug_master;
  `elsif CORE_AXI4Lite
    interface AXI4_Lite_Master_IFC#(DPADDR, DXLEN, 0 ) debug_master;
  `endif
    method Bit#(1) getNDMReset();              // Reset Everything apart from DM & DTM -Active HIGH
    interface Reset dmactive_reset;
  endinterface

  interface Hart_Debug_Ifc;
    method Action   abstractOperation( AbstractRegOp cmd);
    method ActionValue#(Bit#(DXLEN)) abstractReadResponse;
    (*always_enabled,always_ready*)
    method Action   haltRequest(Bit#(1) halt_request);
    (*always_enabled,always_ready*)
    method Action   resumeRequest(Bit#(1) resume_request);
    (*always_enabled,always_ready*)
    method Action   hartReset(Bit#(1) hart_reset_v); // Change to reset type // Signal TO Reset HART -Active HIGH
    (*always_enabled,always_ready*)
    method Action   dm_active(Bit#(1) dm_active);
    (*always_enabled,always_ready*)
    method Bit#(1)  has_reset;
    (*always_enabled,always_ready*)
    method Bit#(1)  is_halted;
    (*always_enabled,always_ready*)
    method Bit#(1)  is_unavailable;
  endinterface

  // Thses rules can fire iff the hart is available where capture that on the debug module side
  // Every interface pairing is a seperate rule to prevent any implict conditions blocking others
  // Abstract Interface has implict conditions , abstract operations are guarded.
  instance Connectable #(Hart_Debug_Ifc,Debug_Hart_Ifc);
    module mkConnection #(Hart_Debug_Ifc hart,Debug_Hart_Ifc debug_module)(Empty);
      
      rule operation;
        let x <- debug_module.abstractOperation;
        hart.abstractOperation(x);
      endrule

      rule response;
        let x <- hart.abstractReadResponse();
        debug_module.abstractReadResponse(x);
      endrule
      
      rule connect_halt_req;
        if(debug_module.dm_active == 1)
          hart.haltRequest(debug_module.haltRequest());
        else
          hart.haltRequest(0);
      endrule

      rule connect_resume_req;
        if(debug_module.dm_active == 1)
          hart.resumeRequest(debug_module.resumeRequest());
        else
          hart.resumeRequest(0);
      endrule

      rule connect_hart_reset;
        if(debug_module.dm_active == 1)
          hart.hartReset(debug_module.hart_reset());
        else 
          hart.hartReset(0);
      endrule
      rule connect_halted;
        debug_module.set_halted(hart.is_halted());
      endrule

      rule connect_available;
        debug_module.set_unavailable(hart.is_unavailable());
      endrule

      rule connect_has_reset;
        debug_module.set_have_reset(hart.has_reset);
      endrule

      rule connect_dm_active;
        hart.dm_active(debug_module.dm_active);
      endrule

    endmodule
  endinstance

  // HART Valid Abstract Access Filter 
  // Each individual register (aside from GPRs) may
  // be supported differently across read, write, and halt status.

  // Target Specific config Defaults for shakti E-Class
  // If Guaranteed non interfereing debug is wanted then add write bit based error
  function Bit#(3) fn_abstract_reg_op_permitted(  Bit#(AbstractAddrWidth) address,
                                                          Bit#(1) halted,
                                                          Bit#(1) abst_ar_write,
                                                          Bit#(3) abst_ar_aarSize);
    // Filter For Valid CSR's and Valid GPR, FPR Access conditiions.
    // E-Class , Registers can be accessed while the hart is running
              // Writes cannot be done to a running hart ,?? are reads permitted ?
              // Access of 32 to DXLEN bit widths are permitted
    Bit#(1) lv_bad_register = 0;    // Register Does not Exist
    Bit#(1) lv_bad_state = 0;   // Hart not in required State
    Bit#(1) lv_bad_size = 0;       // Bad Access Size , essentiall DXLEN Filter

    // Fliter
    if((address >= `FIVO(Abst_reg_address_CSR0)) && (address < `FIVO(Abst_reg_address_GPR0)))begin
      if(address != 14'h07b0)                                     // discriminate basis csr existing
        lv_bad_register = 0;
      else
        lv_bad_register = 0;
    end
    else if((address >= `FIVO(Abst_reg_address_GPR0)) && (address < `FIVO(Abst_reg_address_FPR0)))
      lv_bad_register = 0;
    else if((address >= `FIVO(Abst_reg_address_GPR0)) && (address < (`FIVO(Abst_reg_address_FPR0)+32)))
      lv_bad_register = 0;                                                       //No Floating Point
    else 
      lv_bad_register = 0;//just for testing              //No Implementation Reserved states Mapped 
    
    // State
    if(halted == 0)begin
      if((address >= `FIVO(Abst_reg_address_CSR0)) && (address < `FIVO(Abst_reg_address_GPR0)))
        lv_bad_state = 0;
      else if((address >= `FIVO(Abst_reg_address_GPR0)) && (address < `FIVO(Abst_reg_address_FPR0)))
        lv_bad_state = 0;
      else if((address >= `FIVO(Abst_reg_address_GPR0)) && (address < (`FIVO(Abst_reg_address_FPR0)+32)))
        lv_bad_state = 0;
      else
        lv_bad_state = 0;
    end
    else 
      lv_bad_size = 0;
    
    // Size
    if((abst_ar_aarSize == 3'd2) && ((`FIVO(DXLEN) == 64)||(`FIVO(DXLEN) == 32)))
      lv_bad_size = 0;
    else if ((abst_ar_aarSize == 3'd3) && (`FIVO(DXLEN) == 64))
      lv_bad_size = 0;
    else
      lv_bad_size = 1;

    // Assign Error Type
    if(lv_bad_register == 1)    //Higher Priority of error
      return pack(Abst_Exception);
    else if(lv_bad_state == 1)
      return pack(Abst_WrongState);
    else if(lv_bad_size == 1)
      return pack(Abst_Bus);
    else
      return pack(Abst_NoError);
  endfunction

endpackage
