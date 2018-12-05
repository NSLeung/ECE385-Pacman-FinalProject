module	VGA_Controller(	//	Host Side
						iRed,
						iGreen,
						iBlue,
						iCursor_RGB_EN,
						iCursor_X,
						iCursor_Y,
						iCursor_R,
						iCursor_G,
						iCursor_B,
						oRequest,
						H_Cont_out,
						V_Cont_out,
						//	VGA Side
						oVGA_R,
						oVGA_G,
						oVGA_B,
						oVGA_H_SYNC,
						oVGA_V_SYNC,
						oVGA_SYNC,
						oVGA_BLANK,
						oVGA_CLOCK,
						//	Control Signal
						iCLK,
						iRST_N	);

`include "VGA_Param.h"

//	Host Side
input		[9:0]	iRed;
input		[9:0]	iGreen;
input		[9:0]	iBlue;
input      [3:0]   iCursor_RGB_EN;
input      [9:0]   iCursor_X;
input      [9:0]   iCursor_Y;
input      [9:0]   iCursor_R;
input      [9:0]   iCursor_G;
input      [9:0]   iCursor_B;
output	reg			oRequest;
//	VGA Side
output		[9:0]	oVGA_R;
output		[9:0]	oVGA_G;
output		[9:0]	oVGA_B;
output	reg			oVGA_H_SYNC;
output	reg			oVGA_V_SYNC;
output				oVGA_SYNC;
output				oVGA_BLANK;
output				oVGA_CLOCK;
output	[9:0]		H_Cont_out;
output	[9:0]		V_Cont_out;
//	Control Signal
input				iCLK;
input				iRST_N;

//	Internal Registers and Wires
reg		[9:0]		H_Cont;
reg		[9:0]		V_Cont;
reg		[9:0]		Cur_Color_R;
reg		[9:0]		Cur_Color_G;
reg		[9:0]		Cur_Color_B;
wire				mCursor_EN;
wire				mRed_EN;
wire				mGreen_EN;
wire				mBlue_EN;

assign	H_Cont_out = H_Cont - X_START;
assign	V_Cont_out = V_Cont - Y_START;
assign	oVGA_BLANK	=	oVGA_H_SYNC & oVGA_V_SYNC;
assign	oVGA_SYNC	=	1'b0;
assign	oVGA_CLOCK	=	iCLK;
assign mCursor_EN = iCursor_RGB_EN[3];
assign mRed_EN    = iCursor_RGB_EN[2];
assign mGreen_EN  = iCursor_RGB_EN[1];
assign mBlue_EN   = iCursor_RGB_EN[0];


assign	oVGA_R	=	(	H_Cont>=X_START+130 	&& H_Cont<X_START+H_SYNC_ACT &&
						V_Cont>=Y_START + 60 	&& V_Cont<Y_START+V_SYNC_ACT - 60 )
						?	Cur_Color_R	: (H_Cont>=X_START+60 	&& H_Cont<X_START+120 &&
						V_Cont>=Y_START + 60 	&& V_Cont<Y_START+150)
						? 10'h3FF : (H_Cont>=X_START+60 	&& H_Cont<X_START+120 &&
						V_Cont>=Y_START+330 	&& V_Cont<Y_START+420)? 10'h3FF : 10'h0FF;
						
assign	oVGA_G	=	(	H_Cont>=X_START+130 	&& H_Cont<X_START+H_SYNC_ACT &&
						V_Cont>=Y_START + 60 	&& V_Cont<Y_START+V_SYNC_ACT -60 )
						?	Cur_Color_G	:(H_Cont>=X_START+60 	&& H_Cont<X_START+120 &&
						V_Cont>=Y_START+150 	&& V_Cont<Y_START+240)
						? 10'h3FF : (H_Cont>=X_START+60 	&& H_Cont<X_START+120 &&
						V_Cont>=Y_START+330 	&& V_Cont<Y_START+420)? 10'h3FF : 10'h00F;
						
assign	oVGA_B	=	(	H_Cont>=X_START+130 	&& H_Cont<X_START+H_SYNC_ACT &&
						V_Cont>=Y_START + 60 	&& V_Cont<Y_START+V_SYNC_ACT - 60 )
						?	Cur_Color_B	:(H_Cont>=X_START+60 	&& H_Cont<X_START+120 &&
						V_Cont>=Y_START+240 	&& V_Cont<Y_START+330)
						? 10'h3FF : (H_Cont>=X_START+60 	&& H_Cont<X_START+120 &&
						V_Cont>=Y_START+330 	&& V_Cont<Y_START+420)? 10'h3FF: 10'h0FF;


    // Cursor Generator
    always @(posedge iCLK or negedge iRST_N) begin
        if(!iRST_N) begin
            Cur_Color_R <= 0;
            Cur_Color_G <= 0;
            Cur_Color_B <= 0;
        end
        else begin
            if(H_Cont >= X_START && H_Cont < X_START + H_SYNC_ACT &&
               V_Cont >= Y_START && V_Cont < Y_START + V_SYNC_ACT ) begin
                if( mCursor_EN && (
                    (H_Cont == X_START + iCursor_X) ||
                    //(H_Cont == X_START + iCursor_X + 1) ||
                    (H_Cont == X_START + iCursor_X - 1) ||
                    (V_Cont == Y_START + iCursor_Y) ||
                    //(V_Cont == Y_START + iCursor_Y + 1) ||
                    (V_Cont == Y_START + iCursor_Y - 1)
                )) begin
                    Cur_Color_R <= iCursor_R;
                    Cur_Color_G <= iCursor_G;
                    Cur_Color_B <= iCursor_B;
                end
                else begin
                    Cur_Color_R <= iRed;
                    Cur_Color_G <= iGreen;
                    Cur_Color_B <= iBlue;
                end
            end
            else begin
                Cur_Color_R <= iRed;
                Cur_Color_G <= iGreen;
                Cur_Color_B <= iBlue;
            end
        end
    end
						
//	Pixel LUT Address Generator
always@(posedge iCLK or negedge iRST_N)
begin
	if(!iRST_N)
	oRequest	<=	0;
	else
	begin
		if(	H_Cont>=X_START-2 && H_Cont<X_START+H_SYNC_ACT-2 &&
			V_Cont>=Y_START && V_Cont<Y_START+V_SYNC_ACT )
		oRequest	<=	1;
		else
		oRequest	<=	0;
	end
end

//	H_Sync Generator, Ref. 25.175 MHz Clock
always@(posedge iCLK or negedge iRST_N)
begin
	if(!iRST_N)
	begin
		H_Cont		<=	0;
		oVGA_H_SYNC	<=	0;
	end
	else
	begin
		//	H_Sync Counter
		if( H_Cont < H_SYNC_TOTAL )
		H_Cont	<=	H_Cont+1;
		else
		H_Cont	<=	0;
		//	H_Sync Generator
		if( H_Cont < H_SYNC_CYC )
		oVGA_H_SYNC	<=	0;
		else
		oVGA_H_SYNC	<=	1;
	end
end

//	V_Sync Generator, Ref. H_Sync
always@(posedge iCLK or negedge iRST_N)
begin
	if(!iRST_N)
	begin
		V_Cont		<=	0;
		oVGA_V_SYNC	<=	0;
	end
	else
	begin
		//	When H_Sync Re-start
		if(H_Cont==0)
		begin
			//	V_Sync Counter
			if( V_Cont < V_SYNC_TOTAL )
			V_Cont	<=	V_Cont+1;
			else
			V_Cont	<=	0;
			//	V_Sync Generator
			if(	V_Cont < V_SYNC_CYC )
			oVGA_V_SYNC	<=	0;
			else
			oVGA_V_SYNC	<=	1;
		end
	end
end

endmodule