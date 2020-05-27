//////////////////////////////////////////////////////////////////////////////
//
//  Xilinx, Inc. 2008                 www.xilinx.com
//
//////////////////////////////////////////////////////////////////////////////
//
//  File name :       	serdes_n_to_1.v
//
//  Description :     	1-bit generic n:1 transmitter module
// 			Takes in n bits of data and serialises this to 1 bit
// 			data is transmitted LSB first
// 			0, 1, 2 ......
//
//  Date - revision : 	August 1st 2008 - v 1.0
//
//  Author :          	NJS
//
//  Disclaimer: LIMITED WARRANTY AND DISCLAMER. These designs are
//              provided to you "as is". Xilinx and its licensors make and you
//              receive no warranties or conditions, express, implied,
//              statutory or otherwise, and Xilinx specifically disclaims any
//              implied warranties of merchantability, non-infringement,or
//              fitness for a particular purpose. Xilinx does not warrant that
//              the functions contained in these designs will meet your
//              requirements, or that the operation of these designs will be
//              uninterrupted or error free, or that defects in the Designs
//              will be corrected. Furthermore, Xilinx does not warrantor
//              make any representations regarding use or the results of the
//              use of the designs in terms of correctness, accuracy,
//              reliability, or otherwise.
//
//              LIMITATION OF LIABILITY. In no event will Xilinx or its
//              licensors be liable for any loss of data, lost profits,cost
//              or procurement of substitute goods or services, or for any
//              special, incidental, consequential, or indirect damages
//              arising from the use or operation of the designs or
//              accompanying documentation, however caused and on any theory
//              of liability. This limitation will apply even if Xilinx
//              has been advised of the possibility of such damage. This
//              limitation shall apply not-withstanding the failure of the
//              essential purpose of any limited remedies herein.
//
//  Copyright © 2008 Xilinx, Inc.
//  All rights reserved
//
//////////////////////////////////////////////////////////////////////////////

`timescale 1ps/1ps

module serdes_n_to_1 (ioclk, serdesstrobe, reset, gclk, datain, iob_data_out);

parameter integer SF = 8;

input ioclk;
input serdesstrobe;
input reset;
input gclk;
input [SF-1:0] datain;
output iob_data_out;

wire cascade_di;
wire cascade_do;
wire cascade_ti;
wire cascade_to;
wire [8:0] mdatain;

genvar i ;
generate
  for (i = 0; i <= (SF - 1); i = i + 1)
    begin : loop0
      assign mdatain[i] = datain[i] ;
    end
endgenerate

generate
  for (i = (SF); i <= 8; i = i + 1)
    begin : loop1
      assign mdatain[i] = 1'b0 ;
    end
endgenerate

OSERDES2 #(
  .DATA_WIDTH   (SF),
  .DATA_RATE_OQ ("SDR"),
  .DATA_RATE_OT ("SDR"),
  .SERDES_MODE  ("MASTER"),
  .OUTPUT_MODE  ("DIFFERENTIAL")
) oserdes_m (
  .OQ        (iob_data_out),
  .OCE       (1'b1),
  .CLK0      (ioclk),
  .CLK1      (1'b0),
  .IOCE      (serdesstrobe),
  .RST       (reset),
  .CLKDIV    (gclk),
  .D4        (mdatain[7]),
  .D3        (mdatain[6]),
  .D2        (mdatain[5]),
  .D1        (mdatain[4]),
  .TQ        (),
  .T1        (1'b0),
  .T2        (1'b0),
  .T3        (1'b0),
  .T4        (1'b0),
  .TRAIN     (1'b0),
  .TCE       (1'b1),
  .SHIFTIN1  (1'b1),
  .SHIFTIN2  (1'b1),
  .SHIFTIN3  (cascade_do),
  .SHIFTIN4  (cascade_to),
  .SHIFTOUT1 (cascade_di),
  .SHIFTOUT2 (cascade_ti),
  .SHIFTOUT3 (),
  .SHIFTOUT4 ()
);

OSERDES2 #(
  .DATA_WIDTH   (SF),
  .DATA_RATE_OQ ("SDR"),
  .DATA_RATE_OT ("SDR"),
  .SERDES_MODE  ("SLAVE"),
  .OUTPUT_MODE  ("DIFFERENTIAL")
) oserdes_s (
  .OQ        (),
  .OCE       (1'b1),
  .CLK0      (ioclk),
  .CLK1      (1'b0),
  .IOCE      (serdesstrobe),
  .RST       (reset),
  .CLKDIV    (gclk),
  .D4        (mdatain[3]),
  .D3        (mdatain[2]),
  .D2        (mdatain[1]),
  .D1        (mdatain[0]),
  .TQ        (),
  .T1        (1'b0),
  .T2        (1'b0),
  .T3        (1'b0),
  .T4        (1'b0),
  .TRAIN     (1'b0),
  .TCE       (1'b1),
  .SHIFTIN1  (cascade_di),
  .SHIFTIN2  (cascade_ti),
  .SHIFTIN3  (1'b1),
  .SHIFTIN4  (1'b1),
  .SHIFTOUT1 (),
  .SHIFTOUT2 (),
  .SHIFTOUT3 (cascade_do),
  .SHIFTOUT4 (cascade_to)
);

endmodule
