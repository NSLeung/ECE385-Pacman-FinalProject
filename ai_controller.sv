module  AI_controller ( input         Clk,                // 50 MHz clock
                             Reset,              // Active-high reset signal
                             frame_clk,          // The clock indicating a new frame (~60Hz)
               input [9:0]   DrawX, DrawY,       // Current pixel coordinates
               output logic[7:0] AI_mem_start_X, AI_mem_start_Y,
               output logic  isAI            // Whether current pixel belongs to ball or background
					// ,output logic frame_clk_rising_edge
              );

    parameter [7:0] AI_X_startpos = 8'd58;  // start position for center of AI on the X axis
    parameter [7:0] AI_Y_startpos = 8'd120;  // start position for center of AI on the Y axis
    parameter [7:0] AI_X_Min = 8'd12;       // Left motion coordinate bound
    parameter [7:0] AI_X_Max = 8'd90;     // Right motion coordinate bound
    parameter [7:0] AI_Y_Min = 8'd0;       // Topmost point on the Y axis
    parameter [7:0] AI_Y_Max = 8'd255;     // Bottommost point on the Y axis
    parameter [7:0] AI_X_Step = 8'd1;      // Step size on the X axis
    parameter [7:0] AI_Y_Step = 8'd1;      // Step size on the Y axis

    //collider math
    parameter [7:0] AI_Size = 8'd5;        // Ball size


//    logic [7:0] AI_X_Pos_in, AI_Y_Pos_in;
    logic [7:0] AI_X_Motion_in = 1'b1;
    logic [7:0] AI_Y_Motion_in = 1'b0;
	 // In lab 8 initially the ball moves upwards but here there is no initial movement condition
	 logic [7:0] AI_X_Motion = 8'b1;
	 logic[7:0] AI_Y_Motion = 8'b0;
   logic [7:0] AI_X_Pos_in = AI_X_startpos;
   logic [7:0] AI_Y_Pos_in = AI_Y_startpos;
	 logic[7:0] AI_X_Pos = AI_X_startpos;
	 logic[7:0] AI_Y_Pos = AI_Y_startpos;

    //////// Do not modify the always_ff blocks. ////////
    // Detect rising edge of frame_clk
//    logic frame_clk_delayed;
//	 frame_clk_rising_edge;
//    always_ff @ (posedge Clk) begin
//        frame_clk_delayed <= frame_clk;
//        frame_clk_rising_edge <= (frame_clk == 1'b1) && (frame_clk_delayed == 1'b0);
//    end
    // Update registers
    always_ff @ (posedge frame_clk)
    begin
        if (Reset)
        begin
            AI_X_Pos <= AI_X_startpos;
            AI_Y_Pos <= AI_Y_startpos;
            AI_X_Motion <= 8'd0;
            // AI_X_Motion <= AI_X_Step;
            AI_Y_Motion <= 8'd0;
        end
        else
        begin
          //drive state machine
            AI_X_Pos <= AI_X_Pos_in;
            AI_Y_Pos <= AI_Y_Pos_in;
            AI_X_Motion <= AI_X_Motion_in;
            AI_Y_Motion <= AI_Y_Motion_in;
        end
    end


    // You need to modify always_comb block.
    always_comb
    begin
      AI_X_Pos_in = AI_X_Pos;
      AI_Y_Pos_in = AI_Y_Pos;
      AI_X_Motion_in = AI_X_Motion;
      // AI_Y_Motion_in = AI_Y_Motion;

		  // AI_X_Motion_in = 8'd1; //assigned in if construct?
		  AI_Y_Motion_in = 8'd0; //halt vertical motion for now

      if( AI_X_Pos + AI_Size >= AI_X_Max )  // Ball is at the right edge, BOUNCE!
         AI_X_Motion_in = (~(AI_X_Step) + 1'b1);  // 2's complement.
      else if ( AI_X_Pos <= AI_X_Min + AI_Size )  // Ball is at the left edge, BOUNCE!
         AI_X_Motion_in = AI_X_Step;
      else
        AI_X_Motion_in = AI_X_Motion;
        AI_Y_Motion_in = AI_Y_Motion;

		  // AI_X_Pos_in = AI_X_Pos + 8'd1;
      AI_X_Pos_in = AI_X_Pos + AI_X_Motion;
		  AI_Y_Pos_in = AI_Y_Pos + AI_Y_Motion;
    end

    // Compute whether the pixel corresponds to AI or background
    /* Since the multiplicants are required to be signed, we have to first cast them
       from logic to int (signed by default) before they are multiplied. */

    int DistX, DistY, Size;
    assign DistX = DrawX - AI_X_Pos;
    assign DistY = DrawY - AI_Y_Pos;
    assign Size = AI_Size;

    assign AI_mem_start_X = AI_X_Pos - 8'd5;
 	  assign AI_mem_start_Y = AI_Y_Pos - 8'd5;


    always_comb begin
        // if ( ( DistX*DistX + DistY*DistY) <= (Size*Size) )
        // just absolute value! but uses multiplier
        if ((DistX*DistX <= 8'd25) && (DistY*DistY <= 8'd25))
            isAI = 1'b1;
        else
            isAI = 1'b0;

        /* The ball's (pixelated) circle is generated using the standard circle formula.  Note that while
           the single line is quite powerful descriptively, it causes the synthesis tool to use up three
           of the 12 available multipliers on the chip! */
    end

endmodule
