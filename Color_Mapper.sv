//-------------------------------------------------------------------------
//    Color_Mapper.sv                                                    --
//    Stephen Kempf                                                      --
//    3-1-06                                                             --
//                                                                       --
//    Modified by David Kesler  07-16-2008                               --
//    Translated by Joe Meng    07-07-2013                               --
//    Modified by Po-Han Huang  10-06-2017                               --
//                                                                       --
//    Fall 2017 Distribution                                             --
//                                                                       --
//    For use with ECE 385 Lab 8                                         --
//    University of Illinois ECE Department                              --
//-------------------------------------------------------------------------

// color_mapper: Decide which color to be output to VGA for each pixel.
module  color_mapper (
                        // input              is_ball,            // Whether current pixel belongs to ball
                                                              //   or background (computed in ball.sv)
                                                              //
                    //   SRAM data width is 16 bits wide so we must use this as an input
                      input [1:0] color_index,
                       // input        [9:0] DrawX, DrawY,       // Current pixel coordinates
                       output logic [7:0] VGA_R, VGA_G, VGA_B // VGA RGB output
                     );

    logic [7:0] Red, Green, Blue;

    // Output colors to VGA
    assign VGA_R = Red;
    assign VGA_G = Green;
    assign VGA_B = Blue;

    // Assign color based on is_ball signal
    always_comb
      begin
        //case on color_index to get the respective RGB values
        case(color_index)
          2'd0:
            begin
              //first entry in color pallete is black
              Red = 8'h00;
              Green = 8'h00;
              Blue = 8'h00;
            end
          2'd1:
          begin
            Red = 8'hFA;
            Green = 8'hB9;
            Blue = 8'hB0;
          end
          2'd2:
          begin
            Red = 8'h21;
            Green = 8'h21;
            Blue = 8'hFF;
          end
          2'd3:
          begin
            Red = 8'hFC;
            Green = 8'hB5;
            Blue = 8'hFF;
          end
          default:
          //some default values that we can change later
            begin
            	Red = 8'h7F;
            	Green = 8'h7F;
            	Blue = 8'h7F;
            end
        endcase
      end



    //lab 8 logic
    // begin
    //     if (is_ball == 1'b1)
    //     begin
    //         // White ball
    //         Red = 8'hff;
    //         Green = 8'hff;
    //         Blue = 8'hff;
    //     end
    //     else
    //     begin
    //         // Background with nice color gradient
    //         Red = 8'h3f;
    //         Green = 8'h00;
    //         Blue = 8'h7f - {1'b0, DrawX[9:3]};
    //     end
    // end

endmodule
