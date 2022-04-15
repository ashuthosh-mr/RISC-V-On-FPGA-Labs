# Packaging an IP(AXI Peripheral)

* Create a new project and add the pattern detector .v file. It has a 3 bit pattern to be detected in the 32 bit input. (Refer Figure 1) The output is the number of occurrences of the pattern stored in the 32 bit output register. Note: The pattern detector .v file is verified with post synthesis simulation.
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-30%20161647.png)

* Go to tools > create and package new IP.
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-30%20161824.png)

* Choose the option, create a new AXI4 peripheral.
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-30%20161912.png)

* Give a name and the location of IP where you want it to be on your computer. You will need to know the path of the IP in the upcoming steps.
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-30%20162031.png)

* For this example, default settings i.e 4 slave registers, 32 bit data width with AXI Slave Lite mode is sufficient. ![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-30%20162115.png)

* Choose Edit IP and click the finish button. It will open a new project to package the IP.
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-30%20162157.png)

*  The newly opened project will look like this.
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-30%20162300.png)

* Add the pattern detector .v file to the newly opened project. Next, initialize the pattern detector module in the following line(401) of the pattern_v1_0_S00_AXI.v
Observe that we mapped slv_reg0(slave register 0) to our 32 bit input. 3 bit pattern to slv_reg[2:0] and 32 bit Countout to counter. In the same .v file, we have also initialized the counter signal as 32 bit wire.
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-30%20162854.png)
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-30%20163053.png)

* Change the line(376) from slv_reg3 to counter (i.e mapping slv_reg3 to counter), in order to drive slv_reg3 with the output of the pattern detector.
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-30%20163203.png)

* The project sources should look like this if you have correctly followed the steps.
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-30%20163305.png)

* In the packaging steps, go to file groups, and click on merge changes from file groups wizard.
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-30%20163344.png)

* Click review and package and choose Re-package IP option.
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-30%20163440.png)

* You can close the project after it gives the above message.
![](https://github.com/ashuthosh-mr/RISC-V-On-FPGA-Labs/blob/main/Labs/screenshots/Screenshot%202022-03-30%20163509.png)

* You have now successfully packaged an IP to be used with the Shakti microprocessor.
