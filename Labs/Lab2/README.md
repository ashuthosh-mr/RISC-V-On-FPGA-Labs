Programming the FPGA with Shakti-SoC and running programs on it
===============================================================
In this lab we will program the Shakti-SoC consisting of 32-bit RISC-V microprocessor into the Arty-A7 35T or 100T boards. We will also run C programs on the processor and measure the cycles of the desired part of the code.

Requirements
------------
* MCS file of the SoC
* Vivado 2018 or higher
* Shakti-SDK
* Miniterm UART terminal
* OpenOCD
* Shakti RISC-V toolchain(from lab session 1)

Programming the FPGA board on Vivado
------------------------------------
Download the mcs file provided in this repository. Choose arty35.mcs if you are targeting Arty A7 35T board. Otherwise choose arty100.mcs file. The mcs file contains the information of the hardware design of the SoC that can be programmed to the Flash memory of the board. The design is loaded to the FPGA whenever the FPGA is powered thereafter. The design will remain in the flash memory until it is overwritten again.
<br/>
* Connect the Arty A7-35t board, Open Hardware Manager in Vivado, Open target option and click auto connect. The board name and status will be displayed.
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-31%093906.png)

* Right click on the xc7a35t_0(1) > Add Configuration memory device. Enter the following details to select the flash device present in the Arty A7 board. Choose s25fl128sxxxxxxx0-spi-x1_x2_x4 and click Ok.
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-31%094027.png)

* A window to select the configuration file will open. In the configuration file section, give the path of the mcs file. In my case, it is pattern.mcs. It should be arty35.mcs or arty100.mcs in your case. Fill the checkboxes appropriately and click Ok.
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-31%094147.png)

* Programming the flash with the Shakti SoC is complete.
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-31%094257.png)
