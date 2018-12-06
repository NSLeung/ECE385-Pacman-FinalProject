module draw_control(input logic CLK,
							input logic RESET,
							input logic ENABLE,  //blanking enable for drawing
							// input logic DRAW_READY,

							// input logic [1:0] PLAYER_DIR,
							// input logic [5:0] UI_ENABLE,
							// input logic [5:0] PLAYER_X,
							// input logic [5:0] PLAYER_Y,
							// input logic DRAW_COMMAND,
							// input logic [2:0] MAP_ID,
							output logic [19:0] WRITEADDR,
							output logic [15:0] DATA,
							output logic FD_WE_N,
							output logic [7:0] ocm_data_out
							);


enum logic [3:0] {
	Await,  //0
	Reset,
	Background_Draw,
	Background_ReadData,
	Sleep1,
	Sleep2,  //5
	Background_ReadTile,
	Player_ReadData,
	Player_Sleep,
	Player_Draw,
	UI_ReadData,  //10
	UI_Sleep,
	UI_Draw
} State, Next_State;



assign WRITEADDR = address;
assign DATA = data;
assign FD_WE_N = write_n;
assign ocm_data_out = ocm_data;
endmodule
