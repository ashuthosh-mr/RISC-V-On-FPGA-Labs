`ifdef simulate
`define logLevel (modname, level, log_string) \
        begin \
            let display_all <- $test$plusargs("fullverbose"); \
            let current_module <- $test$plusargs(`"m``modname`"); \
            let current_level <- $test$plusargs(`"l``level`"); \
	          let ____t <- $time; \
            if( display_all || (current_module && current_level)) $display($format("[%10d", ____t) + $format("] ") + log_string); \
        end
`else
  `define logLevel (modname, level, log_string) begin end
`endif
