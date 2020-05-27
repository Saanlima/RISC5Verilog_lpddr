`timescale 1ns / 1ps
// 1024x768 display controller NW/PR 24.1.2014

module VID(
    input pclk, inv,
    input [31:0] viddata,
    output [15:0] vidadr,
    output hsync, vsync,  // to display
    output vde,
    output vid);

//localparam Org = 18'b1101_1111_1111_0000_00;  // DFF00: adr of vcnt=1023
reg [10:0] hcnt;
reg [9:0] vcnt;
reg [31:0] pixbuf;
reg hblank;
wire hend, vend, vblank, xfer;

assign hend = (hcnt == 1327), vend = (vcnt == 805);
assign vblank = (vcnt[8] & vcnt[9]);  // (vcnt >= 768)
assign hsync = ~((hcnt >= 1048+6) & (hcnt < 1185+6));  // -ve polarity
assign vsync = (vcnt >= 771) & (vcnt < 777);  // +ve polarity
assign xfer = (hcnt[4:0] == 6);  // data delay > hcnt cycle + req cycle
assign vid = (pixbuf[0] ^ inv) & ~hblank & ~vblank;
assign vidadr = {1'b0, ~vcnt, hcnt[9:5]} - 16'h2000;
assign vde = ~hblank & ~vblank;

always @(posedge pclk) begin  // pixel clock domain
  hcnt <= hend ? 0 : hcnt+1;
  vcnt <= hend ? (vend ? 0 : (vcnt+1)) : vcnt;
  hblank <= xfer ? hcnt[10] : hblank;  // hcnt >= 1024
  pixbuf <= xfer ? viddata : {1'b0, pixbuf[31:1]};
end

endmodule
