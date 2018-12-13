module draw_control(input logic CLK,
							input logic RESET,
							// input logic ENABLE,  //blanking enable for drawing
							// input logic DRAW_READY,

							input logic [9:0] DrawX,
							input logic [9:0] DrawY,
							input logic[7:0] pac_mem_start_X, pac_mem_start_Y, // provides the start address of pacman png 10x10

							input isPac, isAI,
							// input logic [1:0] PLAYER_DIR,
							// input logic [5:0] UI_ENABLE,
							// input logic [5:0] PLAYER_X,
							// input logic [5:0] PLAYER_Y,
							// input logic DRAW_COMMAND,
							// input logic [2:0] MAP_ID,
							// output logic [19:0] WRITEADDR,
							// output logic [15:0] DATA,
							// output logic FD_WE_N,

							//output address into RAM.sv module
							output logic [15:0] mem_address_out
							);

//state machine
enum logic [1:0] {
	Wait

} State, Next_State;

//local variables
logic [15:0] mem_address;
logic [15:0] bg_address_index, bg_address_index_in;
logic [15:0] pac_address_index, pac_address_index_in;
logic [15:0] AI_address_index, AI_address_index_in;
logic [7:0] pac_mem_pos_X, pac_mem_pos_Y;


//instantiate modules here


//2 ALWAYS CONSTRUCT

//address calculations
always_comb begin
	//always increment bg_address_index
	bg_address_index_in = bg_address_index + 16'b1;

	//check if in bounds
	if((DrawX < 256) && (DrawY < 256) && (isPac == 1'b0))
// if((DrawX < 256) && (DrawY < 256))
	begin
			// mem_address = DrawX + 10'd256 * DrawY;
//			mem_address = 16'd5;
			// bg_address_index_in = bg_address_index + 16'b1;
			mem_address_out = bg_address_index;
	end
	else if (isPac == 1'b1 && isAI == 1'b0)
	begin
		//grab pacman in memory
		// mem_address = pac_mem_pos_X + 8'd10 * pac_mem_pos_Y;
		pac_address_index_in = pac_address_index + 16'b1;
		mem_address_out = pac_address_index;
	end
	else if (isAI == 1'b1 && isPac == 1'b0)
	begin
		//grab AI in memory
		AI_address_index_in = AI_address_index + 16'b1;
		mem_address_out = AI_address_index;
	end
	else
	begin
		mem_address_out = 16'd0;
		//else, reset all back to 0 as frame is done drawing
		bg_address_index_in = 16'b0;
		pac_address_index_in = 16'b0;
		AI_address_index_in = 16'b0;

	end
end

//ALWAYS_FF BLOCK
always_ff @ (posedge CLK)
begin
	if (RESET)
	begin
//		mem_address_out <= 16'b0;
			bg_address_index <= 16'b0;
			pac_address_index <= 16'b0;
			AI_address_index <= 16'b0;
	end
	else
	begin
		bg_address_index_in <= bg_address_index;
		pac_address_index_in <= pac_address_index;
		AI_address_index_in <= AI_address_index;
	end
	//increment bg_counter always
	// bg_address_index <= bg_address_index + 16'b1;
end


//assign WRITEADDR = address;
//assign DATA = data;
//assign FD_WE_N = write_n;
// assign ocm_data_out = ocm_data;
assign pac_mem_pos_X = DrawX - pac_mem_start_X;
assign pac_mem_pos_Y = DrawY - pac_mem_start_Y;

//assign mem_address_out = mem_address;
endmodule
