module DRAM16XN (DATA_IN, ADDRESS, ADDRESS_DP, WRITE_EN, CLK, O_DATA_OUT, O_DATA_OUT_DP);

parameter data_width = 20;

input [data_width-1:0] DATA_IN;
input [3:0] ADDRESS;
input [3:0] ADDRESS_DP;
input WRITE_EN;
input CLK;

output [data_width-1:0] O_DATA_OUT_DP;
output [data_width-1:0] O_DATA_OUT;

genvar i;
generate
  for(i = 0; i < data_width; i = i + 1) begin : dram16s
    RAM16X1D i_RAM16X1D_U(  
      .D     (DATA_IN[i]),
      .WE    (WRITE_EN),
      .WCLK  (CLK),
      .A0    (ADDRESS[0]),
      .A1    (ADDRESS[1]),
      .A2    (ADDRESS[2]),
      .A3    (ADDRESS[3]),
      .DPRA0 (ADDRESS_DP[0]),
      .DPRA1 (ADDRESS_DP[1]),
      .DPRA2 (ADDRESS_DP[2]),
      .DPRA3 (ADDRESS_DP[3]),
      .SPO   (O_DATA_OUT[i]),
      .DPO   (O_DATA_OUT_DP[i])
    );
  end
endgenerate

endmodule

