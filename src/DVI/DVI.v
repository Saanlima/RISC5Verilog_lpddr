`timescale 1 ps / 1 ps

module DVI (
  input  wire reset,
  input  wire pll_locked,
  input  wire clkx1in,
  input  wire clkx2in,
  input  wire clkx10in,
  input  wire [7:0] blue_in,
  input  wire [7:0] green_in,
  input  wire [7:0] red_in,
  input  wire       hsync,
  input  wire       vsync,
  input  wire       vde,
  output wire [3:0] TMDS,
  output wire [3:0] TMDSB
);

wire [9:0] red;
wire [9:0] green;
wire [9:0] blue;

encode encb (
  .clkin  (clkx1in),
  .rstin  (reset),
  .din    (blue_in),
  .c0     (hsync),
  .c1     (vsync),
  .de     (vde),
  .dout   (blue)
);

encode encg (
  .clkin  (clkx1in),
  .rstin  (reset),
  .din    (green_in),
  .c0     (1'b0),
  .c1     (1'b0),
  .de     (vde),
  .dout   (green)
);
  
encode encr (
  .clkin  (clkx1in),
  .rstin  (reset),
  .din    (red_in),
  .c0     (1'b0),
  .c1     (1'b0),
  .de     (vde),
  .dout   (red)
);

wire [4:0] tmds_data0, tmds_data1, tmds_data2;

convert_30to15_fifo pixel2x (
  .rst     (reset),
  .clk     (clkx1in),
  .clkx2   (clkx2in),
  .datain  ({red[9:5], green[9:5], blue[9:5], red[4:0], green[4:0], blue[4:0]}),
  .dataout ({tmds_data2, tmds_data1, tmds_data0})
);

wire pclkx10;
wire serdesstrobe;
wire bufpll_lock;
wire [2:0] tmdsint;
wire serdes_reset = reset | ~bufpll_lock;

BUFPLL #(.DIVIDE(5)) ioclk_buf (.PLLIN(clkx10in), .GCLK(clkx2in), .LOCKED(pll_locked),
                      .IOCLK(pclkx10), .SERDESSTROBE(serdesstrobe), .LOCK(bufpll_lock));


serdes_n_to_1 #(.SF(5)) oserdes0 (
           .ioclk(pclkx10),
           .serdesstrobe(serdesstrobe),
           .reset(serdes_reset),
           .gclk(clkx2in),
           .datain(tmds_data0),
           .iob_data_out(tmdsint[0])
);

serdes_n_to_1 #(.SF(5)) oserdes1 (
           .ioclk(pclkx10),
           .serdesstrobe(serdesstrobe),
           .reset(serdes_reset),
           .gclk(clkx2in),
           .datain(tmds_data1),
           .iob_data_out(tmdsint[1])
);

serdes_n_to_1 #(.SF(5)) oserdes2 (
           .ioclk(pclkx10),
           .serdesstrobe(serdesstrobe),
           .reset(serdes_reset),
           .gclk(clkx2in),
           .datain(tmds_data2),
           .iob_data_out(tmdsint[2])
);

OBUFDS TMDS0 (.I(tmdsint[0]), .O(TMDS[0]), .OB(TMDSB[0]));
OBUFDS TMDS1 (.I(tmdsint[1]), .O(TMDS[1]), .OB(TMDSB[1]));
OBUFDS TMDS2 (.I(tmdsint[2]), .O(TMDS[2]), .OB(TMDSB[2]));

reg [4:0] tmdsclkint = 5'b00000;
reg toggle = 1'b0;

always @ (posedge clkx2in or posedge serdes_reset) begin
  if (serdes_reset)
    toggle <= 1'b0;
  else
    toggle <= ~toggle;
end

always @ (posedge clkx2in) begin
  if (toggle)
    tmdsclkint <= 5'b11111;
  else
    tmdsclkint <= 5'b00000;
end

wire tmdsclk;

serdes_n_to_1 #(
  .SF           (5))
clkout (
  .iob_data_out (tmdsclk),
  .ioclk        (pclkx10),
  .serdesstrobe (serdesstrobe),
  .gclk         (clkx2in),
  .reset        (serdes_reset),
  .datain       (tmdsclkint)
);

OBUFDS TMDS3 (.I(tmdsclk), .O(TMDS[3]), .OB(TMDSB[3]));

endmodule
