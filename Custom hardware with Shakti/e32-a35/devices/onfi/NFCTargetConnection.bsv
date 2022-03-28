package NFCTargetConnection ;

import Connectable :: * ;

import InterfaceNandFlashController :: * ;
import NandFlashFunctBlockTargetTop :: * ;

	module mkConnection #( ONFiInterface onfi_ifc , ONFI_Target_Interface_top target_ifc ) ( Empty ) ;

		//(* no_implicit_conditions, fire_when_enabled *)

		//mkConnection(target_ifc.data0,onfi_ifc.dataio0);
		//mkConnection(target_ifc.data1,onfi_ifc.dataio1);
		
		rule rl_ready_busy_0 ;
			onfi_ifc._ready_busy0_n_m (target_ifc.t_ready_busy_n_0) ;
		endrule
		
		rule rl_onfi_ce_0 ;
			target_ifc._onfi_ce_n_0 (onfi_ifc.onfi_ce0_n_) ;
		endrule
		
		rule rl_onfi_we_0 ;
			target_ifc._onfi_we_n_0 (onfi_ifc.onfi_we_n_) ;
		endrule
		
		rule rl_onfi_re_0 ;
			target_ifc._onfi_re_n_0 (onfi_ifc.onfi_re_n_) ;
		endrule
		
		rule rl_onfi_wp_0 ;
			target_ifc._onfi_wp_n_0 (onfi_ifc.onfi_wp_n_) ;
		endrule

		rule rl_onfi_cle_0 ;
			target_ifc._onfi_cle_0 (onfi_ifc.onfi_cle_) ;
		endrule

		rule rl_onfi_ale_0 ;
			target_ifc._onfi_ale_0 (onfi_ifc.onfi_ale_) ;
		endrule
				
		rule rl_ready_busy_1 ;
			onfi_ifc._ready_busy1_n_m (target_ifc.t_ready_busy_n_1) ;
		endrule
		
		rule rl_onfi_ce_1 ;
			target_ifc._onfi_ce_n_1 (onfi_ifc.onfi_ce1_n_) ;
		endrule
		
		rule rl_onfi_we_1 ;
			target_ifc._onfi_we_n_1 (onfi_ifc.onfi_we_n_) ;
		endrule
		
		rule rl_onfi_re_1 ;
			target_ifc._onfi_re_n_1 (onfi_ifc.onfi_re_n_) ;
		endrule
		
		rule rl_onfi_wp_1 ;
			target_ifc._onfi_wp_n_1 (onfi_ifc.onfi_wp_n_) ;
		endrule

		rule rl_onfi_cle_1 ;
			target_ifc._onfi_cle_1 (onfi_ifc.onfi_cle_) ;
		endrule

		rule rl_onfi_ale_1 ;
			target_ifc._onfi_ale_1 (onfi_ifc.onfi_ale_) ;
		endrule
		
		rule rl_onfi_ENi ;
			target_ifc._onfi_ENi (1'b1) ;
		endrule
		
		rule rl_onfi_Re_c ;
			target_ifc._onfi_Re_c (~onfi_ifc.onfi_re_n_) ;
		endrule
		
	endmodule
endpackage
