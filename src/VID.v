`timescale 1ns / 1ps
// 1024x768 display controller NW/PR 24.1.2014

module VID(
    input reset,
    input pclk, inv,
    output hsync, vsync,     // sync signals
    output vde,              // video display enable
    output [23:0] RGB,
    // interface to VRAM
    output [15:0] vidadr,
    input [31:0] viddata,
    // interface to mcb
    input [23:0] display_c,
    input mcb_clk,
    output reg [23:4] mcb_raddr,
    output reg mcb_rd,
    input [127:0] buff_data,
    input [6:0] buff_addr,
    input buff_wr,
    input mcb_busy,
    input video_mode
);

reg [10:0] hcnt;
reg [9:0] vcnt;
reg hblank;
wire hend, vend, vblank, xfer_m, vid_m;
reg [31:0] pixbuf_m;

wire xfer_c;
wire [15:0] vid_c;
reg [127:0] pixbuf_c;
wire [6:0] vidadr_c;
reg [127:0] viddata_c;
wire vreq;
reg [9:0] vreq_addr;
reg [1:0] state;
reg mcb_busy_sync;

assign hend = (hcnt == 1327), vend = (vcnt == 805);
assign vblank = (vcnt[8] & vcnt[9]);  // (vcnt >= 768)
assign hsync = ~((hcnt >= 1048+6) & (hcnt < 1185+6));  // -ve polarity
assign vsync = (vcnt >= 771) & (vcnt < 777);  // +ve polarity
assign vde = ~hblank & ~vblank;

assign xfer_m = (hcnt[4:0] == 6);  // data delay > hcnt cycle + req cycle
assign vid_m = (pixbuf_m[0] ^ inv) & ~hblank & ~vblank;
assign vidadr = {1'b0, ~vcnt, hcnt[9:5]} - 16'h2000;

assign xfer_c = (hcnt[2:0] == 6);  // data delay > hcnt cycle + req cycle
assign vid_c = pixbuf_c[15:0];
assign vidadr_c = hcnt[9:3];
assign vreq = video_mode & (hcnt == 11'd1024) & ((vcnt < 10'd767) | (vcnt == 10'd805));

assign RGB = video_mode ? 
              {vid_c[15:11], vid_c[15:13], 
               vid_c[10:5], vid_c[10:9],
               vid_c[4:0], vid_c[4:2]} :
              (vid_m ? 24'hffffff : 24'h0);
              
always @(posedge pclk) begin  // pixel clock domain
  hcnt <= hend ? 0 : hcnt+1;
  vcnt <= hend ? (vend ? 0 : (vcnt+1)) : vcnt;
  hblank <= xfer_c ? hcnt[10] : hblank;
  pixbuf_m <= xfer_m ? viddata : {1'b0, pixbuf_m[31:1]};
  pixbuf_c <= xfer_c ? viddata_c : {16'b0, pixbuf_c[127:16]};
end

always @(posedge pclk or posedge reset) begin
  if (reset) begin
    state <= 2'b00;
    mcb_busy_sync <= 1'b0;
    mcb_rd <= 1'b0;
    vreq_addr <= 10'd0;
  end else begin
    mcb_busy_sync <= mcb_busy;
    case(state)
      2'b00: begin
        mcb_rd <= 1'b0;
        vreq_addr <= (vcnt == 10'd805) ? 10'd0 : vcnt + 1'b1;
        mcb_raddr <= display_c[23:4] + 17'h17f80 - {vreq_addr, 7'd0};
        if(vreq) begin
          state <= 2'b01;
        end
      end
      2'b01: begin // read cache from memory
        mcb_rd <= 1'b1;
        if(mcb_busy_sync) begin
          mcb_rd <= 1'b0;
          state <= 2'b10;
        end
      end
      2'b10: begin // wait for memory read to finish
        mcb_rd <= 1'b0;
        if(~mcb_busy_sync)
          state <= 2'b00;
      end
      default: begin
        mcb_rd <= 1'b0;
        state <= 2'b00;
      end
    endcase
  end
end

// scanline buffer
reg [127:0] ram [127:0];

// Port A - write from mcb
always @(posedge mcb_clk) begin
  if(buff_wr) begin
    ram[buff_addr] <= buff_data;
  end
end

// Port B - read from video controller
always @(posedge pclk) begin
  viddata_c <= ram[vidadr_c];
end
  
endmodule
