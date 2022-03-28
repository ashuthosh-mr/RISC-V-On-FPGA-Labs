package bridge_functions;

	typedef enum {RegularReq, BurstReq, ERR} BridgeState deriving(Bits, Eq);
		
	function Tuple3#(Bit#(bytes_out), Bit#(bytes_in), Bit#(data_width_out)) 
																				strb_out(Bit#(bytes_in) in_burst, Bit#(data_width_in) data)
																				provisos(Add#(data_width_out, data_width_in, t_data_width),
																							 	 Div#(data_width_in, 8, bytes_in),
																							   Div#(data_width_out, 8, bytes_out),
																							 	 Max#(bytes_in, bytes_out, bytes_max),
																							 	 Min#(bytes_in, bytes_out, bytes_min),
																							 	 Add#(bytes_in, bytes_out, t_bytes),
																							 	 Add#(a__, bytes_in, bytes_max),
																							 	 Add#(b__, bytes_min, bytes_in),
																							 	 Max#(data_width_in, data_width_out, data_width_max),
																							 	 Min#(data_width_in, data_width_out, data_width_min),
																							   Add#(data_width_max, data_width_min, t_data_width),	
																							 	 Mul#(data_ratio, data_width_min, data_width_max),
																								 Add#(bytes_max, bytes_min, t_bytes)
																								 );
			let v_bytes_in = valueOf(bytes_in);
			let v_bytes_out = valueOf(bytes_out);
			let v_data_min = valueOf(data_width_min);
			let v_bytes_max = valueOf(bytes_max);
			let v_bytes_min = valueOf(bytes_min);
			Bit#(t_data_width) lv_data_out = zeroExtend(data);
			Bit#(t_bytes) burst_out = 0;
			if(valueOf(data_width_out) >= valueOf(data_width_in)) begin
				lv_data_out = zeroExtend(data);
				if(valueOf(bytes_out) >= valueOf(bytes_in)) 
					burst_out = zeroExtend(in_burst);
			end
			else begin 
				Bit#(bytes_max) burst_bytes = zeroExtend(in_burst);
				Bit#(bytes_min) burst_min = truncate(burst_out);
				Bool found_data = False;
				for(Integer i=0; i<valueOf(data_ratio); i=i+1) begin
					burst_bytes =	in_burst[(i+1)*v_bytes_min-1:i*v_bytes_min];
					if(burst_bytes!=0 && !found_data) begin
						found_data = True;
						burst_out = zeroExtend(burst_bytes);
						Bit#(bytes_min) lv_zeroes= 0;
						in_burst[(i+1)*v_bytes_min-1:i*v_bytes_min] = lv_zeroes;
						lv_data_out = data[(i+1)*v_data_min-1:i*v_data_min];	
					end
				end
			end
			return tuple3(truncate(burst_out), in_burst, truncate(lv_data_out));
		endfunction

		function Bit#(data_width_in) read_data_resp(Bit#(data_width_out) data_in,	
																								Bit#(data_width_in) pre_data)
																				provisos(Add#(data_width_out, data_width_in, t_data_width));
			let v_data_wout = valueOf(data_width_out);
			let v_data_win = valueOf(data_width_in);
			//Bit#(TLog#(data_width_in)) data_in_bytes = (1<<arsize_req)*8;
			Bit#(t_data_width) return_data=0;
			return_data[valueOf(t_data_width)-1:valueOf(t_data_width)-v_data_win] = pre_data;
			if(v_data_wout < v_data_win) begin	
					return_data = return_data >> valueOf(data_width_out);
					return_data[valueOf(t_data_width)-1:valueOf(t_data_width)-v_data_wout] = data_in;
					//return_data = zeroExtend(pre_data);
				//end
			end
			return truncateLSB(return_data);
		endfunction

endpackage
