`timescale 1ns / 1ps  // 22.9.2015
// with SRAM, byte access, flt.-pt., and gpio
// PS/2 mouse and network 7.1.2014 PDR

module RISC5Top(
  input sys_clk,
  input SWITCH,
  input  RxD,   // RS-232
  output TxD,
  inout mcb3_rzq,
  output mcb3_dram_we_n,
  inout mcb3_dram_udqs,
  output mcb3_dram_udm,
  output mcb3_dram_ras_n,
  output mcb3_dram_dm,
  inout mcb3_dram_dqs,
  inout [15:0] mcb3_dram_dq,
  output mcb3_dram_ck_n,
  output mcb3_dram_ck,
  output mcb3_dram_cke,
  output mcb3_dram_cas_n,
  output [1:0] mcb3_dram_ba,
  output [12:0] mcb3_dram_a,
  input [1:0] MISO,          // SPI - SD card & network
  output [1:0] SCLK, MOSI,
  output [1:0] SS,
//  output NEN,  // network enable
  output [3:0] TMDS,
  output [3:0] TMDSB,
  input PS2C, PS2D,    // keyboard
  inout msclk, msdat,  // mouse
  output LED1,
  output LED2,
  output LED3,
  output LED4,
  output LED5,
  input [7:0] swi,    // wing c bits 0 - 7
  output [7:0] leds,  // wing c bits 8 - 15
  inout [7:0] gpio    // wing a bits 0 - 7
  );

// IO addresses for input / output
// 0  milliseconds / --
// 1  switches / LEDs
// 2  RS-232 data / RS-232 data (start)
// 3  RS-232 status / RS-232 control
// 4  SPI data / SPI data (start)
// 5  SPI status / SPI control
// 6  PS2 keyboard / --
// 7  mouse / --
// 8  general-purpose I/O data
// 9  general-purpose I/O tri-state control

wire NEN;
wire [3:0] btn = {SWITCH, 3'b000};
//wire [7:0] swi = 8'b10000000;

wire[23:0] adr;
wire [3:0] iowadr;           // word address
wire [31:0] inbus, inbus0;   // data to RISC core
wire [31:0] outbus;          // data from RISC core
wire [31:0] romout, codebus; // code to RISC core
wire rd, wr, ben, ioenb, mreq;

wire [7:0] dataTx, dataRx, dataKbd;
wire rdyRx, doneRx, startTx, rdyTx, rdyKbd, doneKbd;
wire [27:0] dataMs;
reg bitrate;  // for RS232
wire limit;   // of cnt0

reg [7:0] Lreg;
reg [15:0] cnt0;
reg [31:0] cnt1; // milliseconds

wire [31:0] spiRx;
wire spiStart, spiRdy;
reg [3:0] spiCtrl;

reg [23:0] display;
reg [23:0] fg, bg;
wire [23:0] vram_base;
wire vram_access;
wire [31:0] vram_rdata;

wire [15:0] vidadr;
wire [31:0] viddata;

reg [7:0] gpout, gpoc;
wire [7:0] gpin;

wire [23:0] RGB;
wire hsync, vsync, vde;

wire clkfbout, pllclk0, pllclk1, pllclk2;
wire pll_locked;
wire clk;
reg rst;
wire [3:0] ram_be;

PLL_BASE # (
  .CLKIN_PERIOD(20),     // 20ns     =  50 MHz
  .CLKFBOUT_MULT(15),    // 50MHz*15 = 750 MHz
  .CLKOUT0_DIVIDE(1),    // 750/1    = 750 MHz
  .CLKOUT1_DIVIDE(10),   // 750/10   =  75 MHz
  .CLKOUT2_DIVIDE(5),    // 750/5    = 150 MHz
  .COMPENSATION("INTERNAL")
) pll_blk (
  .CLKFBOUT(clkfbout),
  .CLKOUT0(pllclk0),    // 750 MHz
  .CLKOUT1(pllclk1),    // 75 MHz
  .CLKOUT2(pllclk2),    // 150 MHz
  .CLKOUT3(),
  .CLKOUT4(),
  .CLKOUT5(),
  .LOCKED(pll_locked),
  .CLKFBIN(clkfbout),
  .CLKIN(clk50m),
  .RST(1'b0)
  );

BUFG pclkbufg (.I(pllclk1), .O(pclk));
BUFG pclkx2bufg (.I(pllclk2), .O(pclkx2));

RISC5 riscx(.clk(clk), .rst(rst), .ce(CE), .irq(limit),
   .rd(rd), .wr(wr), .ben(ben), .stallX(1'b0),
   .adr(adr), .codebus(codebus), .inbus(inbus),
   .outbus(outbus));

PROM PM (.adr(adr[10:2]), .data(romout), .clk(~clk));

RS232R receiver(.clk(clk), .rst(rst), .RxD(RxD), .fsel(bitrate), .done(doneRx),
   .data(dataRx), .rdy(rdyRx));

RS232T transmitter(.clk(clk), .rst(rst), .start(startTx), .fsel(bitrate),
   .data(dataTx), .TxD(TxD), .rdy(rdyTx));

SPI spi(.clk(clk), .rst(rst), .start(spiStart), .dataTx(outbus),
   .fast(spiCtrl[2]), .dataRx(spiRx), .rdy(spiRdy),
   .SCLK(SCLK[0]), .MOSI(MOSI[0]), .MISO(MISO[0] & MISO[1]));

PS2 kbd(.clk(clk), .rst(rst), .done(doneKbd), .rdy(rdyKbd), .shift(),
   .data(dataKbd), .PS2C(PS2C), .PS2D(PS2D));

MousePM Ms(.clk(clk), .rst(rst), .msclk(msclk), .msdat(msdat), .out(dataMs));

VID video(.pclk(pclk), .inv(swi[7]), .vidadr(vidadr),
   .viddata(viddata), .vid(vid), .hsync(hsync), .vsync(vsync), .vde(vde));

DVI dvi(.clkx1in(pclk), .clkx2in(pclkx2), .clkx10in(pllclk0), .pll_locked(pll_locked),
   .reset(~rst), .red_in(RGB[23:16]), .green_in(RGB[15:8]), .blue_in(RGB[7:0]),
   .hsync(hsync), .vsync(vsync), .vde(vde), .TMDS(TMDS), .TMDSB(TMDSB));

VRAM vram(.clka(~clk), .adra(vram_base[17:2]), .bea(ram_be), .wea(wr & vram_access),
   .wda(outbus), .rda(vram_rdata), .clkb(pclk), .adrb(vidadr),
   .rdb(viddata));

assign RGB = vid ? fg : bg;
assign codebus = adr[23:12] == 12'hFFE ? romout : inbus0;
assign iowadr = adr[5:2];
assign ioenb = (adr[23:6] == 18'h3FFFF);
assign mreq = (adr[23:18] != 6'h3F) & ~vram_access;
assign vram_base = adr[23:0] - display;
assign vram_access = (vram_base[23:18] == 6'h0) & 
        ((vram_base[17] == 1'b0) | ((vram_base[16] == 1'b0) & (vram_base[15] == 1'b0)));

assign inbus = (~ioenb & ~vram_access) ? codebus : (~ioenb & vram_access ? vram_rdata :
   ((iowadr == 0) ? cnt1 :
    (iowadr == 1) ? {20'b0, btn, swi} :
    (iowadr == 2) ? {24'b0, dataRx} :
    (iowadr == 3) ? {30'b0, rdyTx, rdyRx} :
    (iowadr == 4) ? spiRx :
    (iowadr == 5) ? {31'b0, spiRdy} :
    (iowadr == 6) ? {3'b0, rdyKbd, dataMs} :
    (iowadr == 7) ? {24'b0, dataKbd} :
    (iowadr == 8) ? {24'b0, gpin} :
    (iowadr == 9) ? {24'b0, gpoc} :
    (iowadr == 13) ? {8'b0, bg} :
    (iowadr == 14) ? {8'b0, fg} :
    (iowadr == 15) ? {8'b0, display} :
    0));

assign ram_be[0] = ~ben | (~adr[1] & ~adr[0]);
assign ram_be[1] = ~ben | (~adr[1] & adr[0]);
assign ram_be[2] = ~ben | (adr[1] & ~adr[0]);
assign ram_be[3] = ~ben | (adr[1] & adr[0]);

genvar i;
generate // tri-state buffer for gpio port
  for (i = 0; i < 8; i = i+1)
  begin: gpioblock
    IOBUF gpiobuf (.I(gpout[i]), .O(gpin[i]), .IO(gpio[i]), .T(~gpoc[i]));
  end
endgenerate

assign dataTx = outbus[7:0];
assign startTx = wr & ioenb & (iowadr == 2);
assign doneRx = rd & ioenb & (iowadr == 2);
assign limit = (cnt0 == 24999);
assign spiStart = wr & ioenb & (iowadr == 4);
assign SS = ~spiCtrl[1:0];  //active low slave select
assign MOSI[1] = MOSI[0], SCLK[1] = SCLK[0], NEN = spiCtrl[3];
assign doneKbd = rd & ioenb & (iowadr == 7);
assign LED1 = Lreg[0]; 
assign LED2 = Lreg[1]; 
assign LED3 = Lreg[2]; 
assign LED4 = Lreg[3]; 
assign LED5 = ~SS[0];
assign leds = Lreg;

always @(posedge clk)
begin
  rst <= ((cnt1[4:0] == 0) & limit) ? ~btn[3] : rst;
  Lreg <= ~rst ? 0 : (wr & ioenb & (iowadr == 1)) ? outbus[7:0] : Lreg;
  cnt0 <= limit ? 0 : cnt0 + 1;
  cnt1 <= cnt1 + limit;
  spiCtrl <= ~rst ? 0 : (wr & ioenb & (iowadr == 5)) ? outbus[3:0] : spiCtrl;
  bitrate <= ~rst ? 0 : (wr & ioenb & (iowadr == 3)) ? outbus[0] : bitrate;
  gpout <= (wr & ioenb & (iowadr == 8)) ? outbus[7:0] : gpout;
  gpoc <= ~rst ? 0 : (wr & ioenb & (iowadr == 9)) ? outbus[7:0] : gpoc;
  bg <= (wr & ioenb & (iowadr == 13)) ? outbus[23:0] : bg;
  fg <= (wr & ioenb & (iowadr == 14)) ? outbus[23:0] : fg;
  display <= (wr & ioenb & (iowadr == 15)) ? outbus[23:0] : display;
end

initial begin
  display = 24'h0e7f00;
  bg = 24'h0; // 
  fg = 24'hffffff;
end

reg [2:0] state, next_state;
reg [5:0] burst_cnt, next_burst_cnt;
reg [3:0] cache_addr, next_cache_addr;
reg cache_en, next_cache_en;
reg cache_we, next_cache_we;
reg [127:0] cache_wdata, next_cache_wdata;
reg wr_busy, next_wr_busy;
reg rd_busy, next_rd_busy;
wire [127:0] cache_rdata;
wire [23:8] waddr, raddr;
reg mem_wr_sync;
reg mem_rd_sync;

reg c3_p0_cmd_en, next_p0_cmd_en;
reg [2:0] c3_p0_cmd_instr, next_p0_cmd_instr;
reg [29:0] c3_p0_cmd_byte_addr, next_p0_cmd_byte_addr;
reg c3_p0_wr_en, next_p0_wr_en;
reg c3_p0_rd_en, next_p0_rd_en;

wire [127:0] c3_p0_rd_data;
wire [127:0] c3_p0_wr_data;
wire c3_p0_rd_empty;
wire c3_p0_wr_empty;

cache_64k cache (
  .clk(clk),
  .flush_req(vsync),
  .addr(adr[23:0]),
  .dout(inbus0), 
  .din(outbus), 
  .mreq(mreq), 
  .wmask(({4{!ben}} | (1'b1 << adr[1:0])) & {4{wr}}),
  .ce(CE), 
  .mem_din(cache_wdata),
  .mem_dout(cache_rdata),
  .mem_clk(mem_clk),
  .mem_rd(mem_rd),
  .mem_wr(mem_wr),
  .waddr(waddr),
  .raddr(raddr),
  .cache_wr(cache_en && cache_we),
  .cache_rd(cache_en && ~cache_we),
  .cache_addr(cache_addr),
  .rd_busy(rd_busy),
  .wr_busy(wr_busy)
);


assign c3_p0_wr_data = cache_rdata;

parameter [2:0]
  IDLE = 3'h0,
  WRITE1 = 3'h1,
  WRITE2 = 3'h2,
  WRITE3 = 3'h3,
  WRITE4 = 3'h4,
  READ1 = 3'h5,
  READ2 = 3'h6,
  READ3 = 3'h7;

always @ (posedge mem_clk) begin
  if (~c3_calib_done) begin
    state = IDLE;
    burst_cnt = 6'd0;
    cache_addr = 4'd0;
    cache_en = 1'b0;
    cache_wdata = 128'd0;
    cache_we = 1'b0;
    c3_p0_cmd_en = 1'b0;
    c3_p0_cmd_instr = 3'd0;
    c3_p0_cmd_byte_addr = 30'd0;
    c3_p0_wr_en = 1'b0;
    c3_p0_rd_en = 1'b0;
    wr_busy <= 1'b0;
    rd_busy <= 1'b0;
  end else begin
    state = next_state;
    burst_cnt = next_burst_cnt;
    cache_addr = next_cache_addr;
    cache_en = next_cache_en;
    cache_wdata = next_cache_wdata;
    cache_we = next_cache_we;
    c3_p0_cmd_en = next_p0_cmd_en;
    c3_p0_cmd_instr = next_p0_cmd_instr;
    c3_p0_cmd_byte_addr = next_p0_cmd_byte_addr;
    c3_p0_wr_en = next_p0_wr_en;
    c3_p0_rd_en = next_p0_rd_en;
    wr_busy <= next_wr_busy;
    rd_busy <= next_rd_busy;
    mem_wr_sync <= mem_wr;
    mem_rd_sync <= mem_rd;
  end
end

always @* begin
  next_state = state;
  next_burst_cnt = burst_cnt;
  next_cache_addr = cache_en ? cache_addr + 1'b1 : cache_addr;
  next_cache_en = 1'b0;
  next_cache_wdata = c3_p0_rd_data;
  next_cache_we = 1'b0;
  next_p0_cmd_en = 1'b0;
  next_p0_cmd_instr = c3_p0_cmd_instr;
  next_p0_cmd_byte_addr = c3_p0_cmd_byte_addr;
  next_p0_wr_en = 1'b0;
  next_p0_rd_en = 1'b0;
  next_wr_busy = wr_busy;
  next_rd_busy = rd_busy;
  case(state)
    IDLE: begin
      next_wr_busy = 1'b0;
      next_rd_busy = 1'b0;
      if (mem_wr_sync & c3_p0_wr_empty) begin
        next_state = WRITE1;
        next_wr_busy = 1'b1;
        next_cache_addr = 4'd0;
        next_cache_en = 1'b1;
        next_burst_cnt = 6'd0;
        next_p0_cmd_instr = 3'b000;
        next_p0_cmd_byte_addr = {6'd0, waddr, 8'd0};
      end else if (mem_rd_sync) begin
        next_state = READ1;
        next_rd_busy = 1'b1;
        next_cache_addr = 4'd0;
        next_burst_cnt = 6'd0;
        next_p0_cmd_instr = 3'b001;
        next_p0_cmd_en = 1'b1; 
        next_p0_cmd_byte_addr = {6'd0, raddr, 8'd0};
      end
    end
    WRITE1: begin
      next_p0_wr_en = 1'b1;
      next_burst_cnt = burst_cnt + 1'b1;
      if (burst_cnt == 6'd15)
        next_state = WRITE2;
      else begin
        next_cache_en = 1'b1;
      end
    end
    WRITE2: begin
      next_p0_cmd_en = 1'b1;
      next_state = WRITE3;
    end
    WRITE3: begin
      if (c3_p0_wr_empty)
        next_state = IDLE;
    end
    READ1: begin
      if(~c3_p0_rd_empty) begin
        next_p0_rd_en = 1'b1;
        next_state = READ2;
      end
    end
    READ2: begin
      next_p0_rd_en = 1'b1;
      if (c3_p0_rd_en & ~c3_p0_rd_empty) begin
        next_burst_cnt = burst_cnt + 1'b1;
        next_cache_en = 1'b1;
        next_cache_we = 1'b1;
        if (burst_cnt == 6'd15) begin
          next_p0_rd_en = 1'b1;
          next_state = READ3;
        end
      end
    end
    READ3: begin
      next_state = IDLE;
    end
  endcase
end


wire [15:0] c3_p0_wr_mask = 16'b0;
wire [5:0] c3_p0_cmd_bl = 6'd15;
wire c3_p0_cmd_clk = mem_clk;
wire c3_p0_wr_clk = mem_clk;
wire c3_p0_rd_clk = mem_clk;

lpddr lpddr(
  .sys_clk(sys_clk),
  .sys_rst(sys_reset),
  .c3_calib_done(c3_calib_done),
  .clk100m(mem_clk),
  .clk25m(clk),
  .reset(reset),
  .clk50m(clk50m),
  .mcb3_dram_dq(mcb3_dram_dq),
  .mcb3_dram_a(mcb3_dram_a),
  .mcb3_dram_ba(mcb3_dram_ba),
  .mcb3_dram_cke(mcb3_dram_cke),
  .mcb3_dram_ras_n(mcb3_dram_ras_n),
  .mcb3_dram_cas_n(mcb3_dram_cas_n),
  .mcb3_dram_we_n(mcb3_dram_we_n),
  .mcb3_dram_dm(mcb3_dram_dm),
  .mcb3_dram_udqs(mcb3_dram_udqs),
  .mcb3_rzq(mcb3_rzq),
  .mcb3_dram_udm(mcb3_dram_udm),
  .mcb3_dram_dqs(mcb3_dram_dqs),
  .mcb3_dram_ck(mcb3_dram_ck),
  .mcb3_dram_ck_n(mcb3_dram_ck_n),
  .c3_p0_cmd_clk(c3_p0_cmd_clk),
  .c3_p0_cmd_en(c3_p0_cmd_en),
  .c3_p0_cmd_instr(c3_p0_cmd_instr),
  .c3_p0_cmd_bl(c3_p0_cmd_bl),
  .c3_p0_cmd_byte_addr(c3_p0_cmd_byte_addr),
  .c3_p0_cmd_empty(),
  .c3_p0_cmd_full(),
  .c3_p0_wr_clk(c3_p0_wr_clk),
  .c3_p0_wr_en(c3_p0_wr_en),
  .c3_p0_wr_mask(c3_p0_wr_mask),
  .c3_p0_wr_data(c3_p0_wr_data),
  .c3_p0_wr_full(),
  .c3_p0_wr_empty(c3_p0_wr_empty),
  .c3_p0_wr_count(),
  .c3_p0_wr_underrun(),
  .c3_p0_wr_error(),
  .c3_p0_rd_clk(c3_p0_rd_clk),
  .c3_p0_rd_en(c3_p0_rd_en),
  .c3_p0_rd_data(c3_p0_rd_data),
  .c3_p0_rd_full(),
  .c3_p0_rd_empty(c3_p0_rd_empty),
  .c3_p0_rd_count(),
  .c3_p0_rd_overflow(),
  .c3_p0_rd_error()
);

endmodule