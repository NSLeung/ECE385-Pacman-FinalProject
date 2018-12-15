module Dot #(parameter x = 0 , parameter y = 0 ) (
                                input Clk, Reset,
                                input logic [9:0] DrawX, DrawY,
                                input logic [7:0] pac_mem_start_X, pac_mem_start_Y,
                                output logic isEaten
  );

  logic isCollide = 0;
  logic [7:0] pac_X_Pos, pac_Y_Pos;
  logic [7:0] pac_Size = 8'd5;
  assign pac_X_Pos = pac_mem_start_X + 8'd5;
  assign pac_Y_Pos = pac_mem_start_Y + 8'd5;

  always_ff @ (posedge Clk)
  begin
    if(Reset)
      isCollide <= 1'b0;
    else if( pac_X_Pos + pac_Size == x && pac_Y_Pos == y)
        isCollide <= 1'b1;
    else if( pac_Y_Pos + pac_Size == y && pac_X_Pos == x)
        isCollide <= 1'b1;
    else if( pac_Y_Pos - pac_Size == y && pac_X_Pos == x)
        isCollide <= 1'b1;
    else if( pac_X_Pos - pac_Size == x && pac_Y_Pos == y)
        isCollide <= 1'b1;

  end

  // always_comb
  // begin
  //   if( pac_X_Pos + pac_Size >= x)
  //     isCollide = 1'b1;
  //   if( pac_Y_Pos + pac_Size >= y)
  //     isCollide = 1'b1;
  //   if( pac_Y_Pos - pac_Size <= y)
  //     isCollide = 1'b1;
  //   if( pac_X_Pos - pac_Size <= x)
  //     isCollide = 1'b1;
  //   else
  //     isCollide = 1'b0;
  // end

  always_comb begin
    if((isCollide == 1'b1) && ((DrawX >= x) && (DrawX <= x+3)) && ((DrawY >= y) && (DrawY <= y+3)))
//		if((isCollide == 1'b1) &&
      isEaten = 1'b1;
    else
      isEaten = 1'b0;
  end

endmodule
