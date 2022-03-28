// System Verilog dpi files for verilator sim
// This was first demonstrated in rs nikhils bluespec cores on github(flute)
// NOT documented any where else in the entire bluespec ecosystem
  
import "DPI-C" function int  unsigned init_rbb_jtag(byte unsigned dummy);
import "DPI-C" function byte unsigned get_frame(int client_fd);
import "DPI-C" function void          send_tdo(byte tdo , int client_fd);