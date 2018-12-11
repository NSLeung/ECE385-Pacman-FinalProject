//-------------------------------------------------------------------------
//      lab8.sv                                                          --
//      Christine Chen                                                   --
//      Fall 2014                                                        --
//                                                                       --
//      Modified by Po-Han Huang                                         --
//      10/06/2017                                                       --
//                                                                       --
//      Fall 2017 Distribution                                           --
//                                                                       --
//      For use with ECE 385 Lab 8                                       --
//      UIUC ECE Department                                              --
//-------------------------------------------------------------------------


module final_top( input               CLOCK_50,
             input        [3:0]  KEY,          //bit 0 is set up as Reset
             output logic [6:0]  HEX0, HEX1,HEX2,HEX3,
             // VGA Interface
             output logic [7:0]  VGA_R,        //VGA Red
                                 VGA_G,        //VGA Green
                                 VGA_B,        //VGA Blue
             output logic        VGA_CLK,      //VGA Clock
                                 VGA_SYNC_N,   //VGA Sync signal
                                 VGA_BLANK_N,  //VGA Blank signal
                                 VGA_VS,       //VGA virtical sync signal
                                 VGA_HS,       //VGA horizontal sync signal
             // CY7C67200 Interface
             inout  wire  [15:0] OTG_DATA,     //CY7C67200 Data bus 16 Bits
             output logic [1:0]  OTG_ADDR,     //CY7C67200 Address 2 Bits
             output logic        OTG_CS_N,     //CY7C67200 Chip Select
                                 OTG_RD_N,     //CY7C67200 Write
                                 OTG_WR_N,     //CY7C67200 Read
                                 OTG_RST_N,    //CY7C67200 Reset
             input               OTG_INT,      //CY7C67200 Interrupt
             // SDRAM Interface for Nios II Software
             output logic [12:0] DRAM_ADDR,    //SDRAM Address 13 Bits
             inout  wire  [31:0] DRAM_DQ,      //SDRAM Data 32 Bits
             output logic [1:0]  DRAM_BA,      //SDRAM Bank Address 2 Bits
             output logic [3:0]  DRAM_DQM,     //SDRAM Data Mast 4 Bits
             output logic        DRAM_RAS_N,   //SDRAM Row Address Strobe
                                 DRAM_CAS_N,   //SDRAM Column Address Strobe
                                 DRAM_CKE,     //SDRAM Clock Enable
                                 DRAM_WE_N,    //SDRAM Write Enable
                                 DRAM_CS_N,    //SDRAM Chip Select
                                 DRAM_CLK      //SDRAM Clock



                                 // SRAM controls (may be missing a clk signal??)
//                      				output logic [19:0] SRAM_ADDR,	// SRAM Address 20 Bits
//                      				inout wire [15:0] SRAM_DQ,			// SRAM Data 16 Bits
//                      				output logic 	SRAM_CE_N,			// SRAM Chip Enable
//                      									SRAM_LB_N,			// SRAM Lower Byte Enable (?)
//                      									SRAM_OE_N,			// SRAM Output Enable(?)
//                      									SRAM_UB_N,			// SRAM Upper Byte Enable (?)
//                      									SRAM_WE_N			// SRAM Write Enable (?)
                    );

    logic Reset_h, Clk, Reset_ball_h;
    logic [7:0] keycode;

    //logic for sram ctrl module
    logic ready, enable;
    // logic [15:0] SRAM_DATA, toSRAM_ctrl_data;
    logic [9:0]  DrawX, DrawY;
	  logic [19:0] frontbuff_addr, backbuff_addr;
	  // this was not initialised last night
	  logic SRAM_WE_CTRL;

//    assign enable = ~(VGA_BLANK_N | ~FD_write_request_n);  //set enable 0 when the VGA is trying to read, and the frame_drawer wants to write.
//    assign SRAM_WE_CTRL = VGA_BLANK_N | FD_write_request_n; //always read (1) when not blanking (1) and if not writing (1)




    assign Clk = CLOCK_50;
    always_ff @ (posedge Clk) begin
        Reset_h <= ~(KEY[0]);        // The push buttons are active low
		  Reset_ball_h <= ~(KEY[2]);
    end

    //HPI interface logic variables
    logic [1:0] hpi_addr;
    logic [15:0] hpi_data_in, hpi_data_out;
    logic hpi_r, hpi_w, hpi_cs, hpi_reset;

    //address calculations done inside draw_controller
    logic [19:0] pixel_address;
	   logic [19:0] address;

     //logic variables for draw_control and color_mapper
     logic [3:0] color_index;
		logic [3:0] memtest;

    //logic variables for pac_controller
    logic isPac;
    logic [7:0] pac_mem_start_X, pac_mem_start_Y;


    //always comb block
    always_comb
    begin
      //initialize toSRAM_ctrl_data
      // toSRAM_ctrl_data = frontbuff_data;
      //address index into SRAM (pay attention width)
      //CHANGE HERE
//     pixel_address = DrawY_in * 19'd640 + DrawX_in;
		// pixel_address = DrawY_in * 19'd226 + DrawX_in; // Width of image is 226
    //  if (SRAM_WE_CTRL == 1'b1)
    //    address = pixel_address;
    //  else
			// address = 19'd0; // commented the next line because frontbuff is populated in the framebuffer module and this is the game logic which is not required for just the background.
      // address = frontbuff_addr;
    end

    // Interface between NIOS II and EZ-OTG chip
//    hpi_io_intf hpi_io_inst(
//                            .Clk(Clk),
//                            .Reset(Reset_h),
//                            // signals connected to NIOS II
//                            .from_sw_address(hpi_addr),
//                            .from_sw_data_in(hpi_data_in),
//                            .from_sw_data_out(hpi_data_out),
//                            .from_sw_r(hpi_r),
//                            .from_sw_w(hpi_w),
//                            .from_sw_cs(hpi_cs),
//                            .from_sw_reset(hpi_reset),
//                            // signals connected to EZ-OTG chip
//                            .OTG_DATA(OTG_DATA),
//                            .OTG_ADDR(OTG_ADDR),
//                            .OTG_RD_N(OTG_RD_N),
//                            .OTG_WR_N(OTG_WR_N),
//                            .OTG_CS_N(OTG_CS_N),
//                            .OTG_RST_N(OTG_RST_N)
//    );
//
//     // You need to make sure that the port names here match the ports in Qsys-generated codes.
//     pacman_finalproject_wkeycode_soc nios_system(
//                             .clk_clk(Clk),
//                             .reset_reset_n(1'b1),    // Never reset NIOS
//                             .sdram_wire_addr(DRAM_ADDR),
//                             .sdram_wire_ba(DRAM_BA),
//                             .sdram_wire_cas_n(DRAM_CAS_N),
//                             .sdram_wire_cke(DRAM_CKE),
//                             .sdram_wire_cs_n(DRAM_CS_N),
//                             .sdram_wire_dq(DRAM_DQ),
//                             .sdram_wire_dqm(DRAM_DQM),
//                             .sdram_wire_ras_n(DRAM_RAS_N),
//                             .sdram_wire_we_n(DRAM_WE_N),
//                             .sdram_clk_clk(DRAM_CLK),
//                             .keycode_export(keycode),
//                             .otg_hpi_address_export(hpi_addr),
//                             .otg_hpi_data_in_port(hpi_data_in),
//                             .otg_hpi_data_out_port(hpi_data_out),
//                             .otg_hpi_cs_export(hpi_cs),
//                             .otg_hpi_r_export(hpi_r),
//                             .otg_hpi_w_export(hpi_w),
//                             .otg_hpi_reset_export(hpi_reset)
//    );

    // Use PLL to generate the 25MHZ VGA_CLK.
    // You will have to generate it on your own in simulation.
    vga_clk vga_clk_instance(.inclk0(Clk), .c0(VGA_CLK));

    // VGA Controller module
    // VGA_controller vga_controller_instance(.*, .Reset(Reset_h));
    VGA_controller vga_controller_instance(
                            .Reset(Reset_h),
                            .VGA_HS(VGA_HS),
                            .VGA_VS(VGA_VS),
                            .VGA_CLK(VGA_CLK),
                            .VGA_BLANK_N(VGA_BLANK_N),
                            .VGA_SYNC_N(VGA_SYNC_N),

                            //outputs from vga controller
                            .DrawX(DrawX),
														 .DrawY(DrawY)
    );

    // Which signal should be frame_clk?
    // ball ball_instance(.Clk(Clk), .Reset(Reset_ball_h), .frame_clk(VGA_VS), .DrawX(DrawX), .DrawY(DrawY), .keycode(keycode), .is_ball(is_ball));



    //color mapper module
    // color_mapper color_instance(.*);
    color_mapper color_instance(
                      //input
                        .color_index(color_index),


                        //output
                        .VGA_R(VGA_R),
                        .VGA_G(VGA_G),
                        .VGA_B(VGA_B)
    );

    //instantiate sram_ctrl
    // sram_ctrl sram_ctrl_module(
    //                   .clk(Clk),
    //                   .reset_n(~Reset_h),
    //                   .start_n(enable),
    //                   .addr_in(address),
    //                   .data_write(toSRAM_ctrl_data),
    //                   .rw(SRAM_WE_CTRL),  // 1  means read, 0 means write
    //                   //outputs
    //                   .ready(ready),
    //                   .data_read(SRAM_DATA),
    //                   .sram_addr(SRAM_ADDR),
    //                   .we_n(SRAM_WE_N),
    //                   .oe_n(SRAM_OE_N),
    //                   .ce_a_n(SRAM_CE_N),
    //                   .ub_a_n(SRAM_UB_N),
    //                   .lb_a_n(SRAM_LB_N),
    //                   .data_io(SRAM_DQ)
    // );

	 // Commented out frame_drawer for now because it determines the logic and position for the smaller sprites (Trying to get just the background now)
	 /*
    frame_drawer frame_drawer_instance(.CLK(Clk),
													.RESET(Reset_h),
													.ENABLE(VGA_BLANK_N),
													.DRAW_READY(ready),
                          //output
													.WRITEADDR(frontbuff_addr),


                          //COMING FROM SOFTWARE - NOT IMPLEMENTING YET
													// .PLAYER_X(player_coords[15:8]),
													// .PLAYER_Y(player_coords[7:0]),
													// .PLAYER_DIR(player_info[1:0]),
													.UI_ENABLE(1'b1),
													.DATA(fd_data),
													.DRAW_COMMAND(~KEY[3] | player_info[2]),
													.MAP_ID(player_info[6:4]),
													.FD_WE_N(FD_write_request_n),
													.ocm_data_out(temp_data)
    );

*/
    draw_control draw_control_instance(
                  .CLK(Clk),
                  .RESET(Reset_h),
                  // .ENABLE(VGA_BLANK_N),  //blanking enable for drawing
                  // .DRAW_READY(sram_done),
                  .isPac(isPac),
      						.DrawX(DrawX),
      						.DrawY(DrawY),
                  .pac_mem_start_X(pac_mem_X),
                  .pac_mem_start_Y(pac_mem_Y),
                  //outputs from draw_control
                   // .WRITEADDR(frontbuff_addr),
                   // .DATA(frontbuff_data),
                  // .FD_WE_N(FD_write_request_n),
                   .mem_address_out(address)
    );

    //Maze background here
    frameRAM frameRAM_instance(
    										// .write_address(16'b0), //don't care
    										.read_address(address),
    										.we(1'b0), //we always low
    										.Clk(Clk),
    										.data_out(color_index)
                        ,.memdata(memtest)
    );


    //instantiate pac module - determines whether pacman should be drawn or not
    pac_controller pacman(
          .Clk(CLK),
          .Reset(Reset_h),
          .frame_clk(VGA_VS),
          .DrawX(DrawX),
          .DrawY(DrawY),
          .keycode(keycode),
          .pac_mem_start_X(pac_mem_start_X),
          .pac_mem_start_Y(pac_mem_start_Y),
          .isPac(isPac)
    );






    // Display keycode on hex display
    HexDriver hex_inst_0 ({2'b0, color_index[1:0]}, HEX0);
    HexDriver hex_inst_1 ({3'b0, isPac}, Hex1);
    // HexDriver hex_inst_1 (DrawX[3:0], HEX1);
	 HexDriver hex_inst_2 (DrawX[7:4], HEX2);
	 HexDriver hex_inst_3 (memtest, HEX3);


endmodule
