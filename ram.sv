/*
 * ECE385-HelperTools/PNG-To-Txt
 * Author: Rishi Thakkar
 *
 */

module  frameRAM
(
//		input [1:0] data_In,
		// input [15:0] pacman_address, bg_address,
		//
		// take only 1 address
		input [15:0] read_address,
		input we, Clk,

		input isPac,
		//output is a color_index (centralized or not?)
		// output logic [3:0] data_pacman, data_bg
		output logic[3:0] data_out
		, memdata
);


// mem has width of 4 bits and a total of 256x256 addresses (size of pac_bg png)
logic [3:0] mem1 [0:65535];
//mem g=has width of 4 bits and a total of 10x10 addresses (size of pacman)
logic [3:0] mem2 [0:99];

//infers memory
initial
begin
	 $readmemh("C:/Users/Leungi5368/Documents/UIUC/Sophomore\ 2018/ECE\ 385/Quartus\ Files/PacMaze_FinalProject/ECE385-Pacman-FinalProject/PNG\ To\ Hex/On-Chip\ Memory/sprite_bytes/pacbg_256x256.txt", mem1);
	 $readmemh("C:/Users/Leungi5368/Documents/UIUC/Sophomore\ 2018/ECE\ 385/Quartus\ Files/PacMaze_FinalProject/ECE385-Pacman-FinalProject/PNG\ To\ Hex/On-Chip\ Memory/sprite_bytes/pacman_10x10.txt", mem2);

end


//always_ff @ (posedge Clk) begin
////	if (we)
////		mem[write_address] <= data_In;
//	data_Out<= mem[read_address];
//end

//logic to determine which memory toread from
always_comb begin

	if(isPac == 1'b1)
		begin
			data_out = mem2[read_address];
		end
	else
		begin
			data_out = mem1[read_address];
		end
end



// assign data_pacman = mem2[pacman_address];
// assign data_bg = mem1[bg_address];
assign memdata = mem2[5];
//HexDriver hex_inst_3 (mem[27000], HEX3);
endmodule
