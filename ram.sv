/*
 * ECE385-HelperTools/PNG-To-Txt
 * Author: Rishi Thakkar
 *
 */

module  frameRAM
(
//		input [1:0] data_In,
		input [15:0] write_address, read_address,
		input we, Clk,
		
		output logic [3:0] data_Out, memdata
);

//infers memory
// mem has width of 3 bits and a total of 400 addresses
logic [3:0] mem [0:65535];
//logic [1:0] mem[0:56047]

initial
begin
	 $readmemh("C:/Users/Leungi5368/Documents/UIUC/Sophomore\ 2018/ECE\ 385/Quartus\ Files/PacMaze_FinalProject/ECE385-Pacman-FinalProject/PNG\ To\ Hex/On-Chip\ Memory/sprite_bytes/pacbg_256x256.txt", mem);
end


//always_ff @ (posedge Clk) begin
////	if (we)
////		mem[write_address] <= data_In;
//	data_Out<= mem[read_address];
//end

assign data_Out = mem[read_address];
assign memdata = mem[6];
//HexDriver hex_inst_3 (mem[27000], HEX3);
endmodule
