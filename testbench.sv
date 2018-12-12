module testbench_pac();

timeunit 10ns;	// Half clock cycle at 50 MHz
  			// This is the amount of time represented by #1
timeprecision 1ns;

//SIMULATING TOP LEVEL MODULE
// logic Clk;
// logic [3:0]  KEY;
//  logic [6:0]  HEX0, HEX1,HEX2,HEX3;
//  logic [7:0]  VGA_R, VGA_G, VGA_B;
//  logic        VGA_CLK, VGA_SYNC_N, VGA_BLANK_N, VGA_VS, VGA_HS;
//  wire  [15:0] OTG_DATA;     //CY7C67200 Data bus 16 Bits
// logic [1:0]  OTG_ADDR;     //CY7C67200 Address 2 Bits
// logic        OTG_CS_N, OTG_RD_N, OTG_WR_N, OTG_RST_N, OTG_INT;
// logic [12:0] DRAM_ADDR;
// wire  [31:0] DRAM_DQ;
// logic [1:0]  DRAM_BA;
// logic [3:0]  DRAM_DQM;
// logic        DRAM_RAS_N, DRAM_CAS_N, DRAM_CKE, DRAM_WE_N, DRAM_CS_N, DRAM_CLK;

// final_top pacman_proj(.*);

//SIMULATING PAC_CONTROLLER
logic         Clk,                // 50 MHz clock
         Reset,              // Active-high reset signal
         frame_clk;          // The clock indicating a new frame (~60Hz)
logic [9:0]   DrawX, DrawY;      // Current pixel coordinates
logic[7:0]	  keycode;				 // keycode


logic[7:0] pac_mem_start_X, pac_mem_start_Y;
logic  isPac;

pac_controller pac_control(.*);

//internal monitoring signal logic variables
//logic [7:0] keycode;

logic [15:0] mem_address;
//logic [9:0] DrawX, DrawY;
logic [7:0] pac_X_Pos, pac_Y_Pos, pac_X_Pos_in, pac_Y_Pos_in, pac_X_Motion, pac_Y_Motion, pac_X_Motion_in, pac_Y_Motion_in;
logic temp, frame_clk_rising_edge;
always_comb begin: INTERNAL_MONITORING
// mem_address = pacman_proj.draw_control_instance.mem_address;
DrawX = pac_control.DrawX;
DrawY = pac_control.DrawY;
pac_X_Pos = pac_control.pac_X_Pos;
pac_Y_Pos = pac_control.pac_Y_Pos;
pac_X_Pos_in = pac_control.pac_X_Pos_in;
pac_Y_Pos_in = pac_control.pac_Y_Pos_in;
pac_X_Motion = pac_control.pac_X_Motion;
pac_Y_Motion = pac_control.pac_Y_Motion;
pac_X_Motion_in = pac_control.pac_X_Motion_in;
pac_Y_Motion_in = pac_control.pac_Y_Motion_in;
pac_mem_start_X = pac_control.pac_mem_start_X;
pac_mem_start_Y = pac_control.pac_mem_start_Y;
keycode = pac_control.keycode;
temp = pac_control.flag;
// frame_clk_rising_edge = pac_control.frame_clk_rising_edge;
end


always begin : CLOCK_GENERATION
#1 Clk = ~Clk;
#1 frame_clk = ~frame_clk;
end

initial begin: CLOCK_INITIALIZATION
    Clk = 0;
    frame_clk = 0;
end

initial begin: TEST_VECTORS
// #30 force keycode = 8'h1F;
#2 Reset = 1;
#2 Reset = 0;
#2 frame_clk_rising_edge = 1;
#2 frame_clk_rising_edge = 0;
#2 frame_clk_rising_edge = 1;
#2 frame_clk_rising_edge = 0;
end


endmodule
