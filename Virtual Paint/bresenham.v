module gpu_func_draw_line(
    input                             clk,
    input                             reset,
    input                             start,
    output reg                        finished,
    //Function parameter interface.
    input [9:0]             x1,
    input [8:0]             y1,
    input [9:0]             x2,
    input [8:0]             y2,
    //Output interface.
    output reg [9:0]        pos_x,
    output reg [8:0]        pos_y
);
reg	start_after;
reg	[2:0]	state;
wire	tirgger;
wire	[9:0] x_small, x_big;
wire	[8:0]	y_small, y_big;
wire	[10:0]	x_sum;
wire	[9:0]		y_sum;

parameter init = 3'd0, incre = 3'd1, last = 3'd2, final = 3'd3, next = 3'd4;

assign x_sum = x_big + x_small;
assign y_sum = y_big + y_small;
assign x_big = (x1 > x2) ? x1: x2;
assign x_small = (x1 > x2) ? x2: x1;
assign y_big = (y1 > y2) ? y1 : y2;
assign y_small = (y1 > y2) ? y2 : y1;

always @(posedge clk) begin
if(reset) start_after <= 1'b0;
else 	start_after <= start;
end

assign trigger = start && (!start_after);

always @(posedge clk) begin
if(reset) begin
	pos_x <= 10'b0;
	pos_y <= 9'b0;
	finished <= 1'b1;
	state <= init;
end
else 
	case(state)
	init: begin
		if(trigger) begin
			if((x_big < x_small + 20) && (y_big < y_small + 20)) begin
			state <= incre;
			pos_x <= x_small;
			pos_y <= y_small;
			finished <= 1'b0;
			end
			else state <= init;
		end
		else state <= init;
	end
	
	incre: begin
			pos_x <= (x_sum) >> 1;
			pos_y <= (y_sum) >> 1;
			finished <= 1'b0;
			state <= last;
	end
	
	last: begin
			pos_x <= x_big;
			pos_y <= y_big;
			finished <= 1'b0;
			state <= final;
	end
	
	final: begin
			finished <= 1'b1;
			state <= init;
	end
	endcase
end	
endmodule 