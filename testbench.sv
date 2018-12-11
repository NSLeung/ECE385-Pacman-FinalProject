module testbench_pac();

timeunit 10ns;	// Half clock cycle at 50 MHz
  			// This is the amount of time represented by #1
timeprecision 1ns;


logic CLOCK_50;
logic [3:0]  KEY;
 logic [6:0]  HEX0, HEX1,HEX2,HEX3;
 logic [7:0]  VGA_R, VGA_G, VGA_B;
 logic        VGA_CLK, VGA_SYNC_N, VGA_BLANK_N, VGA_VS, VGA_HS;
 wire  [15:0] OTG_DATA;     //CY7C67200 Data bus 16 Bits
logic [1:0]  OTG_ADDR;     //CY7C67200 Address 2 Bits
logic        OTG_CS_N, OTG_RD_N, OTG_WR_N, OTG_RST_N, OTG_INT;
logic [12:0] DRAM_ADDR;
wire  [31:0] DRAM_DQ;
logic [1:0]  DRAM_BA;
logic [3:0]  DRAM_DQM;
logic        DRAM_RAS_N, DRAM_CAS_N, DRAM_CKE, DRAM_WE_N, DRAM_CS_N, DRAM_CLK;

final_top pacman_proj(.*);

logic [15:0] mem_address;

always_comb begin: INTERNAL_MONITORING
mem_address = pacman_proj.draw_control_instance.mem_address;
end


always begin : CLOCK_GENERATION
#1 CLOCK_50 = ~CLOCK_50;
end

initial begin: CLOCK_INITIALIZATION
    CLOCK_50 = 0;
end

initial begin: TEST_VECTORS

end


endmodule
