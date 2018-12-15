module  pac_controller ( input         Clk,                // 50 MHz clock
                             Reset,              // Active-high reset signal
                             frame_clk,          // The clock indicating a new frame (~60Hz)
               input [9:0]   DrawX, DrawY,       // Current pixel coordinates
					     input[7:0]	  keycode,				 // keycode


               output logic[7:0] pac_mem_start_X, pac_mem_start_Y,
               output logic  isPac            // Whether current pixel belongs to ball or background
//					output logic frame_clk_rising_edge
              );

    parameter [7:0] pac_X_startpos = 8'd13;  // start position for center of pacman on the X axis
    parameter [7:0] pac_Y_startpos = 8'd12;  // start position for center of pacman on the Y axis
    parameter [7:0] pac_X_Min = 8'd0;       // Leftmost point on the X axis
    parameter [7:0] pac_X_Max = 8'd255;     // Rightmost point on the X axis
    parameter [7:0] pac_Y_Min = 8'd0;       // Topmost point on the Y axis
    parameter [7:0] pac_Y_Max = 8'd255;     // Bottommost point on the Y axis
    parameter [7:0] pac_X_Step = 8'd1;      // Step size on the X axis
    parameter [7:0] pac_Y_Step = 8'd1;      // Step size on the Y axis

    //collider math
    parameter [7:0] pac_Size = 8'd5;        // Ball size


    logic [7:0] pac_X_Pos_in = pac_X_startpos;
	 logic [7:0] pac_Y_Pos_in = pac_Y_startpos;
	 // In lab 8 initially the ball moves upwards but here there is no initial movement condition
	 logic [7:0] pac_X_Motion = 8'b0;
	 logic[7:0] pac_Y_Motion = 8'b0;
   logic [7:0] pac_X_Motion_in = 8'b0;
   logic [7:0] pac_Y_Motion_in = 8'b0;
	 logic[7:0] pac_X_Pos = pac_X_startpos;
	 logic[7:0] pac_Y_Pos = pac_Y_startpos;

//		keycode = 8'h1F;
	// logic [7:0] temp = 8'h1F;
  // logic flag = 1'b1;
    //////// Do not modify the always_ff blocks. ////////
    // Detect rising edge of frame_clk
    logic frame_clk_delayed, frame_clk_rising_edge;
    always_ff @ (posedge Clk) begin
        frame_clk_delayed <= frame_clk;
        frame_clk_rising_edge <= (frame_clk == 1'b1) && (frame_clk_delayed == 1'b0);
    end
    // Update registers
    always_ff @ (posedge frame_clk)
    begin
        if (Reset)
        begin
            pac_X_Pos <= pac_X_startpos;
            pac_Y_Pos <= pac_Y_startpos;
            pac_X_Motion <= 8'd0;
            // pac_X_Motion <= pac_X_Step;
            pac_Y_Motion <= 8'd0;
        end
        else
        begin
            pac_X_Pos <= pac_X_Pos_in;
            pac_Y_Pos <= pac_Y_Pos_in;
            pac_X_Motion <= pac_X_Motion_in;
            pac_Y_Motion <= pac_Y_Motion_in;
        end
    end
    //////// Do not modify the always_ff blocks. ////////

    // You need to modify always_comb block.
    always_comb
    begin
        // By default, keep motion and position unchanged
        pac_X_Pos_in = pac_X_Pos;
        pac_Y_Pos_in = pac_Y_Pos;
		  // By default (if no key is pressed), pacman must not move
        // pac_X_Motion_in = 0;
        // pac_Y_Motion_in = 0;

        pac_X_Motion_in = pac_X_Motion;
        pac_Y_Motion_in = pac_Y_Motion;
        // Update position and motion only at rising edge of frame clock
//        if (frame_clk_rising_edge)
//        begin
            // Be careful when using comparators with "logic" datatype because compiler treats
            //   both sides of the operator as UNSIGNED numbers.
            // e.g. Ball_Y_Pos - Ball_Size <= Ball_Y_Min
            // If Ball_Y_Pos is 0, then Ball_Y_Pos - Ball_Size will not be -4, but rather a large positive number.


				//begin keycode logic
				unique case(keycode)
					// 'W' (UP)
					8'h1A:
						begin
							//clear horizontal motion for any vertical motion
							pac_X_Motion_in = 8'd0;
							if( pac_Y_Pos + pac_Size >= pac_Y_Max )  // pacman is at the bottom edge, stay there!
								 pac_Y_Motion_in = 8'd0;  // Y Motion is 0 too so that it stays in same place.
							else if ( pac_Y_Pos <= pac_Y_Min + pac_Size )  // pacman is at the top edge, stay there!
								 pac_Y_Motion_in = 8'd0;
							else if( pac_X_Pos + pac_Size >= pac_X_Max )	//pacman is at right edge, stay there!
									begin
										pac_X_Motion_in = 8'd0;
										pac_Y_Motion_in = 8'd0; //clear y_motion;
									end
							else if ( pac_X_Pos <= pac_X_Min + pac_Size ) //pacman is at left edge, stay there!
									begin
										pac_X_Motion_in = 8'd0;
										pac_Y_Motion_in = 8'd0; //clear y_motion;
									end
							else
							//continue in upward DIRECTION
								pac_Y_Motion_in = ~(8'd1) + 1'b1; // -1

						end


					// 'S' (DOWN)
					8'h16:
						begin
							//clear horizontal motion for any vertical motion
							pac_X_Motion_in = 8'd0;
							if( pac_Y_Pos + pac_Size >= pac_Y_Max )  // pacman is at the bottom edge, stay there!
								 pac_Y_Motion_in = 8'd0;  // Y Motion is 0 too so that it stays in same place.
							else if ( pac_Y_Pos <= pac_Y_Min + pac_Size )  // pacman is at the top edge, stay there!
								 pac_Y_Motion_in = 8'd0;
							else if( pac_X_Pos + pac_Size >= pac_X_Max )	//pacman is at right edge, stay there!
									begin
										pac_X_Motion_in = 8'd0;
										pac_Y_Motion_in = 8'd0; //clear y_motion;
									end
							else if ( pac_X_Pos <= pac_X_Min + pac_Size ) //pacman is at left edge, stay there!
									begin
										pac_X_Motion_in = pac_X_Step;
										pac_Y_Motion_in = 8'd0; //clear y_motion;
									end
							else
							//continue in downward DIRECTION
								pac_Y_Motion_in = 8'd1; // +1

						end


					// 'a' (LEFT)
					8'h04:
						begin
							//clear vertical motion for any horizontal motion
							pac_Y_Motion_in = 8'd0;
							if( pac_Y_Pos + pac_Size >= pac_Y_Max )  // pacman is at the bottom edge, stay there!
								 pac_Y_Motion_in = 8'd0;  // Y Motion is 0 too so that it stays in same place.
							else if ( pac_Y_Pos <= pac_Y_Min + pac_Size )  // pacman is at the top edge, stay there!
								 pac_Y_Motion_in = 8'd0;
							else if( pac_X_Pos + pac_Size >= pac_X_Max )	//pacman is at right edge, stay there!
									begin
										pac_X_Motion_in = 8'd0;
										pac_Y_Motion_in = 8'd0; //clear y_motion;
									end
							else if ( pac_X_Pos <= pac_X_Min + pac_Size ) //pacman is at left edge, stay there!
									begin
										pac_X_Motion_in = 8'd0;
										pac_Y_Motion_in = 8'd0; //clear y_motion;
									end
							else
							//continue in left DIRECTION
								pac_X_Motion_in = ~(8'd1) + 1'b1; // -1

						end

					// 'd' (RIGHT)
					8'h07:
						begin
							//clear vertical motion for any horizontal motion
							pac_Y_Motion_in = 8'd0;
							if( pac_Y_Pos + pac_Size >= pac_Y_Max )  // pacman is at the bottom edge, stay there!
								 pac_Y_Motion_in = 8'd0;  // Y Motion is 0 too so that it stays in same place.
							else if ( pac_Y_Pos <= pac_Y_Min + pac_Size )  // pacman is at the top edge, stay there!
								 pac_Y_Motion_in = 8'd0;
							else if( pac_X_Pos + pac_Size >= pac_X_Max )	//pacman is at right edge, stay there!
									begin
										pac_X_Motion_in = 8'd0;
										pac_Y_Motion_in = 8'd0; //clear y_motion;
									end
							else if ( pac_X_Pos <= pac_X_Min + pac_Size ) //pacman is at left edge, stay there!
									begin
										pac_X_Motion_in = 8'd0;
										pac_Y_Motion_in = 8'd0; //clear y_motion;
									end
							else
							//continue in right DIRECTION
								pac_X_Motion_in = 8'd1; // 1

						end

						//default case?
						default:
							begin
							//clear vertical motion for any horizontal motion
//							Ball_Y_Motion_in = 8'd0;
							if( pac_Y_Pos + pac_Size >= pac_Y_Max )  // pacman is at the bottom edge, stay there!
								 pac_Y_Motion_in = 8'd0;  // Y Motion is 0 too so that it stays in same place.
							else if ( pac_Y_Pos <= pac_Y_Min + pac_Size )  // pacman is at the top edge, stay there!
								 pac_Y_Motion_in = 8'd0;
							else if( pac_X_Pos + pac_Size >= pac_X_Max )	//pacman is at right edge, stay there!
									begin
										pac_X_Motion_in = 8'd0;
										pac_Y_Motion_in = 8'd0; //clear y_motion;
									end
							else if ( pac_X_Pos <= pac_X_Min + pac_Size ) //pacman is at left edge, stay there!
									begin
										pac_X_Motion_in = 8'd0;
										pac_Y_Motion_in = 8'd0; //clear y_motion;
									end
							else

								begin
									pac_X_Motion_in = 8'd0;
									// pac_X_Motion_in = pac_X_Motion;
//
//									//don't want it to move like in lab 8
									// pac_Y_Motion_in = pac_Y_Motion;
                  pac_Y_Motion_in = 8'd0;
								end
							end

							// TODO: Add other boundary detections and handle keypress here.



//						end
					endcase
					// Update the ball's position with its motion
							// pac_X_Pos_in = pac_X_Pos + pac_X_Motion;
//              pac_X_Pos_in = pac_X_Pos + 8'd1;
//
//							pac_Y_Pos_in = pac_Y_Pos + pac_Y_Motion;
//              flag = 1'b0;
//        end
		  pac_X_Pos_in = pac_X_Pos + pac_X_Motion;
			pac_Y_Pos_in = pac_Y_Pos + pac_Y_Motion;
//              flag = 1'b0;
    end

    // Compute whether the pixel corresponds to pacman or background
    /* Since the multiplicants are required to be signed, we have to first cast them
       from logic to int (signed by default) before they are multiplied. */

    int DistX, DistY, Size;
    assign DistX = DrawX - pac_X_Pos;
    assign DistY = DrawY - pac_Y_Pos;
    assign Size = pac_Size;

    assign pac_mem_start_X = pac_X_Pos - 8'd5;
 	  assign pac_mem_start_Y = pac_Y_Pos - 8'd5;


    always_comb begin
        // if ( ( DistX*DistX + DistY*DistY) <= (Size*Size) )
        // just absolute value! but uses multiplier
        if ((DistX*DistX <= 8'd25) && (DistY*DistY <= 8'd25))
            isPac = 1'b1;
        else
            isPac = 1'b0;

        /* The ball's (pixelated) circle is generated using the standard circle formula.  Note that while
           the single line is quite powerful descriptively, it causes the synthesis tool to use up three
           of the 12 available multipliers on the chip! */
    end

endmodule
