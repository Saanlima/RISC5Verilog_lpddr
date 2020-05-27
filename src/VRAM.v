module VRAM (
  input clka,
  input [15:0] adra,
  input [3:0] bea,
  input wea,
  input [31:0] wda,
  output [31:0] rda,
  input clkb,
  input [15:0] adrb,
  output [31:0] rdb);
  
  wire [7:0] rda_0, rda_1, rda_2, rda_3;
  wire [7:0] viddata_0, viddata_1, viddata_2, viddata_3;

  VRAM8 vram8_0(.clka(clka), .adra(adra), .wea(wea & bea[0]),
  .wda(wda[7:0]), .rda(rda_0), .clkb(clkb), .adrb(adrb),
  .rdb(viddata_0));

  VRAM8 vram8_1(.clka(clka), .adra(adra), .wea(wea & bea[1]),
  .wda(wda[15:8]), .rda(rda_1), .clkb(clkb), .adrb(adrb),
  .rdb(viddata_1));

  VRAM8 vram8_2(.clka(clka), .adra(adra), .wea(wea & bea[2]),
  .wda(wda[23:16]), .rda(rda_2), .clkb(clkb), .adrb(adrb),
  .rdb(viddata_2));

  VRAM8 vram8_3(.clka(clka), .adra(adra), .wea(wea & bea[3]),
  .wda(wda[31:24]), .rda(rda_3), .clkb(clkb), .adrb(adrb),
  .rdb(viddata_3));

  assign rda = {rda_3, rda_2, rda_1, rda_0};
  assign rdb = {viddata_3, viddata_2, viddata_1, viddata_0};

endmodule


module VRAM8 (
  input clka,
  input [15:0] adra,
  input wea,
  input [7:0] wda,
  output reg [7:0] rda,
  input clkb,
  input [15:0] adrb,
  output reg [7:0] rdb);

  reg [7:0] ram [40959:0];

  // Port A
  always @(posedge clka) begin
    rda <= ram[adra];
    if(wea) begin
      rda <= wda;
      ram[adra] <= wda;
    end
  end

  // Port B
  always @(posedge clkb) begin
    rdb <= ram[adrb];
  end
 
endmodule
