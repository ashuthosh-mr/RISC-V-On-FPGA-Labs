Running a C program using RISC-V spike simulator
================================================
In this lab, we will compile a C code using RISC-V toolchain and run the executable using Spike simulator.

Requirements
------------
* Ubuntu 18.04 or later
* Shakti RISC-V toolchain


Dependencies
-----------
Device tree compiler:

    $ sudo apt-get install device-tree-compiler  



Download and install of Shakti RISC-V toolchain
-----------------------------------------------
Download the toolchain. This might take 10-15 minutes depending on the internet speed.  

    $ git clone --recursive https://gitlab.com/shaktiproject/software/shakti-tools.git  

Set the path variable with the path of shakti-tools downloaded in the previous step:

    $ SHAKTITOOLS=/home/user/path/to/shakti-tools
    $ export PATH=$PATH:$SHAKTITOOLS/bin
    $ export PATH=$PATH:$SHAKTITOOLS/riscv64/bin
    $ export PATH=$PATH:$SHAKTITOOLS/riscv64/riscv64-unknown-elf/bin
    $ export PATH=$PATH:$SHAKTITOOLS/riscv32/bin
    $ export PATH=$PATH:$SHAKTITOOLS/riscv32/riscv32-unknown-elf/bin

Open a new terminal, add the path in the .bashrc so that your computer knows the path to shakti-tools hereafter:

    $ gedit .bashrc

Paste the following at the end of the file, save and close:

    SHAKTITOOLS=/home/user/path/to/shakti-tools
    export PATH=$PATH:$SHAKTITOOLS/bin
    export PATH=$PATH:$SHAKTITOOLS/riscv64/bin
    export PATH=$PATH:$SHAKTITOOLS/riscv64/riscv64-unknown-elf/bin
    export PATH=$PATH:$SHAKTITOOLS/riscv32/bin
    export PATH=$PATH:$SHAKTITOOLS/riscv32/riscv32-unknown-elf/bin

Running a hello world program
-----------------------------
Open a new file with name hello.c:

    $ gedit hello.code

Use the following code to print hello world and to print cycles taken to print hello world:

```c
#include <stdio.h>

unsigned long read_cycles(void)
{
unsigned long cycles;
asm volatile ("rdcycle %0" : "=r" (cycles));
return cycles;
}

void main() {
  unsigned long start,end;
  start=read_cycles(); //cycle count at this line is registered in start
  printf("Hello World\n");
  end=read_cycles(); //cycle count at this line is register in end
  printf("Cycles to print hello world is: %lu",end-start); //subtraction gives the cycle count between two lines

}
```
Compile the code using riscv64-unknown-elf-gcc compiler:

    $ riscv64-unknown-elf-gcc hello.c -o hello.out

Simulate the executable using spike:

    $ spike $(which pk) hello.out

The output from the spike simulator:

```
bbl loader
Hello World
Cycles to print hello world is: 45
```

Exercise
--------
* Write a C code to add two arrays of size 10 elements each to get an output of size 10 elements. Print the cycles taken to compute the output.
Note: Measuring of cycles should include only compute of output elements. One can print the output elements to verify outside the measurement.
