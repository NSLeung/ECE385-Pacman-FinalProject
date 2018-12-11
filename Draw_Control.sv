module draw_control(input logic CLK,
							input logic RESET,
							// input logic ENABLE,  //blanking enable for drawing
							// input logic DRAW_READY,

							input logic [9:0] DrawX,
							input logic [9:0] DrawY,
							input logic[7:0] pac_mem_start_X, pac_mem_start_Y, // provides the start address of pacman png 10x10

							input isPac,
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
logic [7:0] pac_mem_pos_X, pac_mem_pos_Y;


//instantiate modules here


//2 ALWAYS CONSTRUCT

//address calculations
always_comb begin
	if(RESET)
	begin
		mem_address = 16'b0;
	end
	//check if in bounds
	else if((DrawX < 256) && (DrawY < 256) && (isPac == 1'b0))
// if((DrawX < 256) && (DrawY < 256))
	begin
			mem_address = DrawX + 10'd256 * DrawY;
//			mem_address = 16'd5;
	end
	else if (isPac)
	begin
		//grab pacman in memory
		mem_address = pac_mem_pos_X + 8'd10 * pac_mem_pos_Y;
	end
	else
	begin
		mem_address = 16'd000;
	end
end



//assign WRITEADDR = address;
//assign DATA = data;
//assign FD_WE_N = write_n;
// assign ocm_data_out = ocm_data;
assign pac_mem_pos_X = DrawX - pac_mem_start_X;
assign pac_mem_pos_Y = DrawY - pac_mem_start_Y;

assign mem_address_out = mem_address;
endmodule
