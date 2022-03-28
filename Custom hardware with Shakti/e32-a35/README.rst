#######
E32-A35
#######

Memory MAP
^^^^^^^^^^

=======  ============  =============
Config   base-address  bound-address
=======  ============  =============
Memory   'h8000_0000   'h8001_FFFF
Memory2  'h8003_0000   'h8003_FFFF
BootRAM  'h0000_1000   'h0000_2FFF
UART0    'h0001_1300   'h0000_1340
UART1    'h0001_1400   'h0000_1440
UART3    'h0001_1500   'h0000_1540
SPI0     'h0002_0000   'h0002_00FF
SPI1     'h0002_0100   'h0002_01FF
PWM0     'h0003_0000   'h0003_00FF
PWM1     'h0003_0100   'h0003_01FF
PWM2     'h0003_0200   'h0003_02FF
PWM3     'h0003_0300   'h0003_03FF
PWM4     'h0003_0400   'h0003_04FF
PWM5     'h0003_0500   'h0003_05FF
I2C0     'h0004_0000   'h0004_00FF
GPIO     'h0004_0100   'h0004_01FF
XADC     'h0004_1000   'h0004_13FF
I2C1     'h0004_1400   'h0004_14FF
PinMux   'h0004_1500   'h0004_15FF
Clint    'h0200_0000   'h020B_FFFF
PLIC     'h0C00_0000   'h0C01_001F
=======  ============  =============

PinMuxing Rules
^^^^^^^^^^^^^^^

=========== ========= =========  ========
Soc signal  Config-0  Config-1   Config-2
=========== ========= =========  ========
io7_cell    GPIO[0]   UART1-RX   -
io8_cell    GPIO[1]   UART1-TX   -
io9_cell    GPIO[2]   UART2-RX   -
io10_cell   GPIO[3]   UART3-TX   PWM_0
io12_cell   GPIO[5]   -          PWM_1
io13_cell   GPIO[6]   -          PWM_2
io16_cell   GPIO[9]   -          PWM_3
io17_cell   GPIO[10]  SPI1_NCS   PWM_4
io18_cell   GPIO[11]  SPI1_MOSI  PWM_5
io19_cell   GPIO[12]  SPI1_MISO  -
io20_cell   GPIO[13]  SPI1_CLK   -
=========== ========= =========  ========

FPGA Board Connector Mapping
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

==========  ============= =============
soc_signal  Schematic Pin Name FPGA Pin
==========  ============= =============
i2c0_sda    jb_p[1]       E15
i2c0_scl    jb_n[1]       E16
gpio_14     jb_p[2]       D15
gpio_15     jb_n[2]       C15
uart0_SOUT  uart_rxd_out  D10
uart0_SIN   uart_txd_in   A9
io7_cell    ck_io[0]      V15
io8_cell    ck_io[1]      U16
io9_cell    ck_io[2]      P14
io10_cell   ck_io[3]      T11
gpio_4      ck_io[4]      R12
io12_cell   ck_io[5]      T14
io13_cell   ck_io[6]      T15
gpio_7      ck_io[7]      T16
gpio_8      ck_io[8]      N15
io16_cell   ck_io[9]      M16
io17_cell   ck_io[10]     V17
io18_cell   ck_io[11]     U18
io19_cell   ck_io[12]     R17
io20_cell   ck_io[13]     P17
gpio_16     led[4]        H5
gpio_17     led[5]        J5
gpio_18     led[6]        T9
gpio_19     led[7]        T10
gpio_20     btn[0]        D9
gpio_21     btn[1]        C9
gpio_22     btn[2]        B9
gpio_23     btn[3]        B8
gpio_24     jd[1]         D4
gpio_25     jd[2]         D3
gpio_26     jd[3]         F4
gpio_27     jd[4]         F3
gpio_28     jd[7]         E2
gpio_29     jd[8]         D2
gpio_30     jd[9]         H2
gpio_31     jd[10]        G2
vauxn4      ck_an_n[0]    C5
vauxp4      ck_an_p[0]    C6
vauxn5      ck_an_n[1]    A5
vauxp5      ck_an_p[1]    A6
vauxn6      ck_an_n[2]    B4
vauxp6      ck_an_p[2]    C4
vauxn7      ck_an_n[3]    A1
vauxp7      ck_an_p[3]    B1
vauxn15     ck_an_n[4]    B2
vauxp15     ck_an_p[4]    B3
vauxn0      ck_an_n[5]    C14
vauxp0      ck_an_p[5]    D14
vauxp12     ad_p[12]      B7
vauxn12     ad_n[12]      B6
vauxp13     ad_p[13]      E6
vauxn13     ad_n[13]      E5
vauxp14     ad_p[14]      A4
vauxn14     ad_n[14]      A3
spi0_nss    qspi_cs       L13
spi0_mosi   qspi_dq[0]    K17
spi0_miso   qspi_dq[1]    K18
i2c1_scl    ck_scl        L18
i2c1_sda    ck_sda        M18
sys_rst     ck_rst        C2
==========  ============= =============



The serial communication happens using uart0 connected to the FPGA package pins D10 and A9, which 
communicate to the host system through the micro-USB port (connector J10).

The debug interface of the processor is connected to the Xilinx JTAG tap, which in-turn is time 
multiplexed with uart0, and is connected to the micro-USB port. This configuration let’s us to 
not have dedicated JTAG pins, thereby eliminating the need for an external JTAG Debug probe (like J-Link).


To Build MCS File:
^^^^^^^^^^^^^^^^^

.. code-block:: shell-session

  $ git clone https://gitlab.com/shaktiproject/gc2020.git
  $ cd gc2020/e32-a35/
  $ make generate_verilog generate_boot_files ip_build arty_build generate_mcs program_mcs JOBS=<jobs>

Connecting to the Target
^^^^^^^^^^^^^^^^^^^^^^^^
Please make sure you are using the 32-bit toolchain available `here
<https://gitlab.com/shaktiproject/software/shakti-tools/>`_. 
(This toolchain contains the ``openocd`` binary which has been tweaked for our bscan2e based JTAG
protocol)

In a New Terminal window

.. code-block:: yaml

  $ openocd -f shakti-arty.cfg

In yet Another Terminal window

.. code-block:: yaml

  $ riscv32-unknown-elf-gdb -x gdb.script

Launch UART Console

.. code-block:: yaml

  $ sudo miniterm /dev/ttyUSB1 19200

On pressing the ``reset-button`` on the board the UART console should display the following:

.. code-block:: yaml

                                       ./((*
                                   ,(((((,
                               *((((((,
                          ./(((((((,
                      ./((((((((*
                   *(((((((((/
               .(((((((((((,
            ,((((((((((((/
          ((((((((((((((/
         .((((((((((((((/
             *(((((((((((.
                  /(((((((.
                ,.     *(((/
                    *((,     ,/.
                      ((((((/.
                       ((((((((((/
                        (((((((((((((/
                        ((((((((((((((.
                       ((((((((((((/
                     *((((((((((*
                   ((((((((((.
                /((((((((*
             *(((((((,
          *((((((.
      .(((((.
  ./(((*
  .
                    SHAKTI PROCESSORS

