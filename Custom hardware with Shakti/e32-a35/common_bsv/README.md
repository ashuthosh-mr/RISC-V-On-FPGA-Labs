# common_bsv

Contains all common BSV modules and templates used across projects.

# Logger.bsv

A Bluespec Macro library to ease logging.

The file exports - 
    
    `logLevel(module_name, log_level, display_string)
    Args:
	module_name: The name of the module which has the event
        log_level : The log level of the event
        display_string : The string to display in the message

Please note, the params to this macro are read up to the comma,
so ``logLevel( stage1,2, ...)`` will work but ``logLevel( stage1 , 2 ...)`` will not work
as the parameter interpreted by Bluespec will be ``2<space>``.

To use this, you need to have the following line in your file - 
    
    `include "Logger.bsv"

Once the executable is created, you can use the following command line args
to control the log displayed - 
    
    ./verilator_executable +m<module name> +l2 +l3

The number corresponds to the ``log_level`` defined in the print statement.

Example usage (TestModule.bsv) - 

    `include "Logger.bsv"
    module testModule ();
	String test=""
        Reg#(Bit#(32) x_val <- mkReg(0);
        
        rule print_every_cycle;
            `logLevel( test, 2, $format("Value of reg is %x", x_val))
        endrule // print_every_cycle
        
        rule update_every_cycle;
            x_val <- x_val + 1;
            if (x_val == 10) $finish;
        endrule
    endmodule // testModule

If you would like to display all statements across the design use +fullverbose
argument -

    ./verilator_executable +fullverbose
