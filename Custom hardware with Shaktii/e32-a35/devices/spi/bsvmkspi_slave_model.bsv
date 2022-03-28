/* 
Copyright (c) 2018, IIT Madras All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions
  and the following disclaimer.  
* Redistributions in binary form must reproduce the above copyright notice, this list of 
  conditions and the following disclaimer in the documentation and/or other materials provided 
 with the distribution.  
* Neither the name of IIT Madras  nor the names of its contributors may be used to endorse or 
  promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------------------------
*/
// Bluespec wrapper, created by Import BVI Wizard
// Created on: Tue May 07 16:15:27 IST 2019
// Created by: vishvesh
// Bluespec version: 2018.10.beta1 2018-10-17 e1df8052c


interface Ifc_spi_model;
	(*always_ready , always_enabled*)
	method Action iss (bit ss);
	(*always_ready , always_enabled*)
	method Action isclk (bit sclk);
	(*always_ready , always_enabled*)
	method Action imosi (bit mosi);
	(*always_enabled*)
	method bit omiso ();
endinterface

import "BVI" spi_slave_model =
module mkspi_slave_model  (Ifc_spi_model);

	parameter Tp = 1;

	default_clock clk_clk;
	default_reset rst_rst;

	input_clock clk_clk (clk)  <- exposeCurrentClock;
	input_reset rst_rst (rst) clocked_by(clk_clk)  <- exposeCurrentReset;


	method iss (ss )
		 enable((*inhigh*)iss_enable) clocked_by(clk_clk) reset_by(rst_rst);
	method isclk (sclk )
		 enable((*inhigh*)isclk_enable) clocked_by(clk_clk) reset_by(rst_rst);
	method imosi (mosi )
		 enable((*inhigh*)imosi_enable) clocked_by(clk_clk) reset_by(rst_rst);
	method miso omiso ()
		 clocked_by(clk_clk) reset_by(rst_rst);

	schedule iss C iss;
	schedule iss CF isclk;
	schedule iss CF imosi;
	schedule iss CF omiso;
	schedule isclk C isclk;
	schedule isclk CF imosi;
	schedule isclk CF omiso;
	schedule imosi C imosi;
	schedule imosi CF omiso;
	schedule omiso CF omiso;
endmodule


