//////////////////////////////////////////////////////////////////////////////////
//
// Filename: cache_64k.v
// Description: 2-way, 128-set cache with 256 byte cache lines
// Version 1.1
// Creation date: May 21, 2020
//
// Author: Magnus Karlsson 
// e-mail: magnus@saanlima.com
//
// Loosely based on cache_controller.v written by Nicolae Dumitrache
//
/////////////////////////////////////////////////////////////////////////////////
// 
// Copyright (C) 2020 Magnus Karlsson
// 
// This source file may be used and distributed without 
// restriction provided that this copyright statement is not 
// removed from the file and that any derivative work contains 
// the original copyright notice and the associated disclaimer.
// 
// This source file is free software; you can redistribute it 
// and/or modify it under the terms of the GNU Lesser General 
// Public License as published by the Free Software Foundation;
// either version 2.1 of the License, or (at your option) any 
// later version. 
// 
// This source is distributed in the hope that it will be 
// useful, but WITHOUT ANY WARRANTY; without even the implied 
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
// PURPOSE. See the GNU Lesser General Public License for more 
// details. 
// 
// You should have received a copy of the GNU Lesser General 
// Public License along with this source; if not, download it 
// from https://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html
// 
///////////////////////////////////////////////////////////////////////////////////

 
module cache_64k(
    input clk,
    input auto_flush_en,
    input flush,
    // cpu interface
    input [23:0] addr,       // cpu address
    output [31:0] dout,      // read data to cpu
    input [31:0] din,        // write data from cpu
    input mreq,              // memory accress request
    input [3:0] wmask,       // memory byte write enable
    output ce,               // cpu clock enable signal (cpu stall if 0)
    // memory controller interface
    input mem_clk,           // memory controller clock
    input [127:0] mem_din,   // read data from memory
    output [127:0] mem_dout, // write data to memory
    output reg mem_rd,       // memory read request
    output reg mem_wr,       // memory write request
    output reg [23:8] waddr, // memory write address
    output [23:8] raddr,     // memory read address
    input cache_wr,          // cache write
    input cache_rd,          // cache read
    input [3:0] cache_addr,  // cache address
    input rd_busy,           // memory controller busy read
    input wr_busy            // memory controller busy write
  );

  reg [8:0] tag [0:255] = 
    {9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 
     9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 
     9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 
     9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 
     9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 
     9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 
     9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 
     9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0,
     9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1,
     9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 
     9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 
     9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 
     9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 
     9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 
     9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 
     9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1, 9'd1} ;
  reg [255:0] dirty = 256'b0;
  reg [127:0] lru = 128'hffffffffffffffffffffffffffffffff;

  wire [6:0] index;
  wire [7:0] target;
  wire hit0, hit1, hit;
  wire st0;
  wire wr;
  reg [1:0] STATE = 2'b00;
  reg [9:0] flush_timer = 10'd0;
  reg flush_sync = 1'b0;
  reg flush_sync_d = 1'b0;
  reg full_flush = 1'b0;
  reg auto_flush = 1'b0;
  reg flushing = 1'b0;
  reg [7:0] flush_target = 8'd0;
  reg rd_busy_sync, wr_busy_sync;

  assign ce = st0 & (~mreq | (hit & ~full_flush & ~auto_flush));
  assign raddr = addr[23:8];
  assign index = addr[14:8];
  assign hit0 = (tag[{1'b0, index}] == addr[23:15]);
  assign hit1 = (tag[{1'b1, index}] == addr[23:15]);
  assign hit = hit0 | hit1;
  assign st0 = (STATE == 2'b00);
  assign wr = |wmask;
  assign target = {lru[index], index};

  always @(posedge clk) begin
    rd_busy_sync <= rd_busy;
    wr_busy_sync <= wr_busy;
    flush_timer <= auto_flush_en ? flush_timer + 1'b1 : 10'd0;
    flush_sync <= flush;
    flush_sync_d <= flush_sync;
    full_flush <= ~auto_flush_en & (full_flush | (flush_sync & ~flush_sync_d));
    auto_flush <= auto_flush_en & (auto_flush | (flush_timer == 10'd1023));
    case(STATE)
      2'b00: begin
        mem_wr <= 1'b0;
        mem_rd <= 1'b0;
        flushing <= 1'b0;
        flush_target <= (~full_flush & ~auto_flush_en) ? 8'd0 : flush_target;
        if (full_flush | auto_flush) begin
          waddr <= {tag[flush_target], flush_target[6:0]}; 
          if (dirty[flush_target]) begin
            flushing <= 1'b1;
            mem_wr <= 1'b1;
            dirty[flush_target] <= 1'b0;
            STATE <= 2'b01;
          end else begin
            auto_flush <= 1'b0;
            full_flush <= full_flush & (flush_target != 8'd255);
            flush_target <= flush_target + 1'b1;
          end
        end else if (mreq) begin
          if (hit0) begin // cache hit0
            lru[index] <= 1'b1;
            if (wr) dirty[{1'b0, index}] <= 1'b1;
          end else if (hit1) begin // cache hit1
            lru[index] <= 1'b0;
            if (wr) dirty[{1'b1, index}] <= 1'b1;
          end else begin  // cache miss
            waddr <= {tag[target], index}; 
            tag[target] <= addr[23:15];
            if (dirty[target]) begin
              mem_wr <= 1'b1;
              dirty[target] <= 1'b0;
              STATE <= 2'b01;
            end else begin
              mem_rd <= 1'b1;
              STATE <= 2'b10;
            end
          end
        end
      end
      2'b01: begin  // write cache to memory
        if (wr_busy_sync) begin
          mem_wr <= 1'b0;
          mem_rd <= ~flushing;
          STATE <= flushing ? 2'b11 : 2'b10;
        end
      end
      2'b10: begin // read cache from memory
        if (rd_busy_sync) begin
          mem_rd <= 1'b0;
          STATE <= 2'b11;
        end
      end
      2'b11: begin // wait for memory controller to finish
        if (~rd_busy_sync & ~wr_busy_sync) begin
          if (flushing) begin
            flushing <= 1'b0;
            auto_flush <= 1'b0;
            full_flush <= full_flush & (flush_target != 8'd255);
            flush_target <= flush_target + 1'b1;
          end
          STATE <= 2'b00;
        end
      end
    endcase
  end

  cache_mem mem
  (
    .clka(~clk),
    .ena(mreq & hit & st0 & ~auto_flush & ~full_flush),
    .wea({4{wr}} & wmask),
    .addra({hit1, addr[14:2]}),
    .dina(din),
    .douta(dout),
    .clkb(mem_clk),
    .enb(cache_wr | cache_rd),
    .web({16{cache_wr}}),
    .addrb({(flushing ? flush_target : target), cache_addr}),
    .dinb(mem_din),
    .doutb(mem_dout)
  );

endmodule

module cache_mem (
    input wire clka,
    input wire ena,
    input wire [3:0] wea,
    input wire [13:0] addra,
    input wire [31:0] dina,
    output wire [31:0] douta,
    input wire clkb,
    input wire enb,
    input wire [15:0] web,
    input wire [11:0] addrb,
    input wire [127:0] dinb,
    output wire [127:0] doutb
  );

  wire [127:0] dina_64;
  wire [127:0] douta_64;
  wire [15:0] wea_64;
  wire [11:0] addra_64;

  assign dina_64 = {4{dina}};
  assign addra_64 = addra[13:2];
  assign wea_64 = addra[1:0] == 2'b00 ? {12'b0, wea} :
                   addra[1:0] == 2'b01 ? {8'b0, wea, 4'b0} :
                   addra[1:0] == 2'b10 ? {4'b0, wea, 8'b0} : {wea, 12'b0};
  assign douta = addra[1:0] == 2'b00 ? douta_64[31:0] :
                 addra[1:0] == 2'b01 ? douta_64[63:32] :
                 addra[1:0] == 2'b10 ? douta_64[95:64] : douta_64[127:96];
                 
  bram_tdp byte0 (
    .a_clk     (clka),
    .a_en      (ena),
    .a_wr      (wea_64[0]),
    .a_addr    (addra_64),
    .a_din     (dina_64[7:0]),
    .a_dout    (douta_64[7:0]),
    .b_clk     (clkb),
    .b_en      (enb),
    .b_wr      (web[0]),
    .b_addr    (addrb),
    .b_din     (dinb[7:0]),
    .b_dout    (doutb[7:0])
  );

  bram_tdp byte1 (
    .a_clk     (clka),
    .a_en      (ena),
    .a_wr      (wea_64[1]),
    .a_addr    (addra_64),
    .a_din     (dina_64[15:8]),
    .a_dout    (douta_64[15:8]),
    .b_clk     (clkb),
    .b_en      (enb),
    .b_wr      (web[1]),
    .b_addr    (addrb),
    .b_din     (dinb[15:8]),
    .b_dout    (doutb[15:8])
  );

  bram_tdp byte2 (
    .a_clk     (clka),
    .a_en      (ena),
    .a_wr      (wea_64[2]),
    .a_addr    (addra_64),
    .a_din     (dina_64[23:16]),
    .a_dout    (douta_64[23:16]),
    .b_clk     (clkb),
    .b_en      (enb),
    .b_wr      (web[2]),
    .b_addr    (addrb),
    .b_din     (dinb[23:16]),
    .b_dout    (doutb[23:16])
  );

  bram_tdp byte3 (
    .a_clk     (clka),
    .a_en      (ena),
    .a_wr      (wea_64[3]),
    .a_addr    (addra_64),
    .a_din     (dina_64[31:24]),
    .a_dout    (douta_64[31:24]),
    .b_clk     (clkb),
    .b_en      (enb),
    .b_wr      (web[3]),
    .b_addr    (addrb),
    .b_din     (dinb[31:24]),
    .b_dout    (doutb[31:24])
  );

  bram_tdp byte4 (
    .a_clk     (clka),
    .a_en      (ena),
    .a_wr      (wea_64[4]),
    .a_addr    (addra_64),
    .a_din     (dina_64[39:32]),
    .a_dout    (douta_64[39:32]),
    .b_clk     (clkb),
    .b_en      (enb),
    .b_wr      (web[4]),
    .b_addr    (addrb),
    .b_din     (dinb[39:32]),
    .b_dout    (doutb[39:32])
  );

  bram_tdp byte5 (
    .a_clk     (clka),
    .a_en      (ena),
    .a_wr      (wea_64[5]),
    .a_addr    (addra_64),
    .a_din     (dina_64[47:40]),
    .a_dout    (douta_64[47:40]),
    .b_clk     (clkb),
    .b_en      (enb),
    .b_wr      (web[5]),
    .b_addr    (addrb),
    .b_din     (dinb[47:40]),
    .b_dout    (doutb[47:40])
  );

  bram_tdp byte6 (
    .a_clk     (clka),
    .a_en      (ena),
    .a_wr      (wea_64[6]),
    .a_addr    (addra_64),
    .a_din     (dina_64[55:48]),
    .a_dout    (douta_64[55:48]),
    .b_clk     (clkb),
    .b_en      (enb),
    .b_wr      (web[6]),
    .b_addr    (addrb),
    .b_din     (dinb[55:48]),
    .b_dout    (doutb[55:48])
  );

  bram_tdp byte7 (
    .a_clk     (clka),
    .a_en      (ena),
    .a_wr      (wea_64[7]),
    .a_addr    (addra_64),
    .a_din     (dina_64[63:56]),
    .a_dout    (douta_64[63:56]),
    .b_clk     (clkb),
    .b_en      (enb),
    .b_wr      (web[7]),
    .b_addr    (addrb),
    .b_din     (dinb[63:56]),
    .b_dout    (doutb[63:56])
  );
  bram_tdp byte8 (
    .a_clk     (clka),
    .a_en      (ena),
    .a_wr      (wea_64[8]),
    .a_addr    (addra_64),
    .a_din     (dina_64[71:64]),
    .a_dout    (douta_64[71:64]),
    .b_clk     (clkb),
    .b_en      (enb),
    .b_wr      (web[8]),
    .b_addr    (addrb),
    .b_din     (dinb[71:64]),
    .b_dout    (doutb[71:64])
  );

  bram_tdp byte9 (
    .a_clk     (clka),
    .a_en      (ena),
    .a_wr      (wea_64[9]),
    .a_addr    (addra_64),
    .a_din     (dina_64[79:72]),
    .a_dout    (douta_64[79:72]),
    .b_clk     (clkb),
    .b_en      (enb),
    .b_wr      (web[9]),
    .b_addr    (addrb),
    .b_din     (dinb[79:72]),
    .b_dout    (doutb[79:72])
  );

  bram_tdp byte10 (
    .a_clk     (clka),
    .a_en      (ena),
    .a_wr      (wea_64[10]),
    .a_addr    (addra_64),
    .a_din     (dina_64[87:80]),
    .a_dout    (douta_64[87:80]),
    .b_clk     (clkb),
    .b_en      (enb),
    .b_wr      (web[10]),
    .b_addr    (addrb),
    .b_din     (dinb[87:80]),
    .b_dout    (doutb[87:80])
  );

  bram_tdp byte11 (
    .a_clk     (clka),
    .a_en      (ena),
    .a_wr      (wea_64[11]),
    .a_addr    (addra_64),
    .a_din     (dina_64[95:88]),
    .a_dout    (douta_64[95:88]),
    .b_clk     (clkb),
    .b_en      (enb),
    .b_wr      (web[11]),
    .b_addr    (addrb),
    .b_din     (dinb[95:88]),
    .b_dout    (doutb[95:88])
  );
  bram_tdp byte12 (
    .a_clk     (clka),
    .a_en      (ena),
    .a_wr      (wea_64[12]),
    .a_addr    (addra_64),
    .a_din     (dina_64[103:96]),
    .a_dout    (douta_64[103:96]),
    .b_clk     (clkb),
    .b_en      (enb),
    .b_wr      (web[12]),
    .b_addr    (addrb),
    .b_din     (dinb[103:96]),
    .b_dout    (doutb[103:96])
  );

  bram_tdp byte13 (
    .a_clk     (clka),
    .a_en      (ena),
    .a_wr      (wea_64[13]),
    .a_addr    (addra_64),
    .a_din     (dina_64[111:104]),
    .a_dout    (douta_64[111:104]),
    .b_clk     (clkb),
    .b_en      (enb),
    .b_wr      (web[13]),
    .b_addr    (addrb),
    .b_din     (dinb[111:104]),
    .b_dout    (doutb[111:104])
  );

  bram_tdp byte14 (
    .a_clk     (clka),
    .a_en      (ena),
    .a_wr      (wea_64[14]),
    .a_addr    (addra_64),
    .a_din     (dina_64[119:112]),
    .a_dout    (douta_64[119:112]),
    .b_clk     (clkb),
    .b_en      (enb),
    .b_wr      (web[14]),
    .b_addr    (addrb),
    .b_din     (dinb[119:112]),
    .b_dout    (doutb[119:112])
  );

  bram_tdp byte15 (
    .a_clk     (clka),
    .a_en      (ena),
    .a_wr      (wea_64[15]),
    .a_addr    (addra_64),
    .a_din     (dina_64[127:120]),
    .a_dout    (douta_64[127:120]),
    .b_clk     (clkb),
    .b_en      (enb),
    .b_wr      (web[15]),
    .b_addr    (addrb),
    .b_din     (dinb[127:120]),
    .b_dout    (doutb[127:120])
  );

endmodule


module bram_tdp #(
    parameter DATA = 8,
    parameter ADDR = 12
  ) (
    // Port A
    input   wire                a_clk,
    input   wire                a_en,
    input   wire                a_wr,
    input   wire    [ADDR-1:0]  a_addr,
    input   wire    [DATA-1:0]  a_din,
    output  reg     [DATA-1:0]  a_dout,
     
    // Port B
    input   wire                b_clk,
    input   wire                b_en,
    input   wire                b_wr,
    input   wire    [ADDR-1:0]  b_addr,
    input   wire    [DATA-1:0]  b_din,
    output  reg     [DATA-1:0]  b_dout
  );
   
  // Shared memory
  reg [DATA-1:0] mem [(2**ADDR)-1:0];
   
  // Port A
  always @(posedge a_clk) begin
    if (a_en) begin
      a_dout <= mem[a_addr];
      if (a_wr) begin
        a_dout <= a_din;
        mem[a_addr] <= a_din;
      end
    end
  end
   
  // Port B
  always @(posedge b_clk) begin
    if (b_en) begin
      b_dout <= mem[b_addr];
      if (b_wr) begin
        b_dout <= b_din;
        mem[b_addr] <= b_din;
      end
    end
  end
 
endmodule