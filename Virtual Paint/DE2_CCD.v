// --------------------------------------------------------------------
// Copyright (c) 2005 by Terasic Technologies Inc. 
// --------------------------------------------------------------------
//
// Permission:
//
//   Terasic grants permission to use and modify this code for use
//   in synthesis for all Terasic Development Boards and Altera Development 
//   Kits made by Terasic.  Other use of this code, including the selling 
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  Terasic provides no warranty regarding the use 
//   or functionality of this code.
//
// --------------------------------------------------------------------
//           
//                     Terasic Technologies Inc
//                     356 Fu-Shin E. Rd Sec. 1. JhuBei City,
//                     HsinChu County, Taiwan
//                     302
//
//                     web: http://www.terasic.com/
//                     email: support@terasic.com
//
// --------------------------------------------------------------------
//
// Major Functions:	DE2 CMOS Camera Demo
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date :| Changes Made:
//   V1.0 :| Johnny Chen       :| 06/01/06  :|      Initial Revision
//   V1.1 :| Johnny Chen       :| 06/02/06  :|      Modify Image Quality
//   V1.2 :| Johnny Chen       :| 06/03/22  :|      Change Pin Assignment For New Sensor
//   V1.3 :| Abhishek	       :| 11/25/10  :|      Virtual Paint Development
// --------------------------------------------------------------------

module DE2_CCD
	(
		////////////////////	Clock Input	 	////////////////////	 
		CLOCK_27,						//	27 MHz
		CLOCK_50,						//	50 MHz
		EXT_CLOCK,						//	External Clock
		////////////////////	Push Button		////////////////////
		KEY,							//	Pushbutton[3:0]
		////////////////////	DPDT Switch		////////////////////
		SW,								//	Toggle Switch[17:0]
		////////////////////	7-SEG Dispaly	////////////////////
		HEX0,							//	Seven Segment Digit 0
		HEX1,							//	Seven Segment Digit 1
		HEX2,							//	Seven Segment Digit 2
		HEX3,							//	Seven Segment Digit 3
		HEX4,							//	Seven Segment Digit 4
		HEX5,							//	Seven Segment Digit 5
		HEX6,							//	Seven Segment Digit 6
		HEX7,							//	Seven Segment Digit 7
		////////////////////////	LED		////////////////////////
		LEDG,							//	LED Green[8:0]
		LEDR,							//	LED Red[17:0]
		////////////////////////	UART	////////////////////////
		UART_TXD,						//	UART Transmitter
		UART_RXD,						//	UART Receiver
		////////////////////////	IRDA	////////////////////////
		IRDA_TXD,						//	IRDA Transmitter
		IRDA_RXD,						//	IRDA Receiver
		/////////////////////	SDRAM Interface		////////////////
		DRAM_DQ,						//	SDRAM Data bus 16 Bits
		DRAM_ADDR,						//	SDRAM Address bus 12 Bits
		DRAM_LDQM,						//	SDRAM Low-byte Data Mask 
		DRAM_UDQM,						//	SDRAM High-byte Data Mask
		DRAM_WE_N,						//	SDRAM Write Enable
		DRAM_CAS_N,						//	SDRAM Column Address Strobe
		DRAM_RAS_N,						//	SDRAM Row Address Strobe
		DRAM_CS_N,						//	SDRAM Chip Select
		DRAM_BA_0,						//	SDRAM Bank Address 0
		DRAM_BA_1,						//	SDRAM Bank Address 0
		DRAM_CLK,						//	SDRAM Clock
		DRAM_CKE,						//	SDRAM Clock Enable
		////////////////////	Flash Interface		////////////////
		FL_DQ,							//	FLASH Data bus 8 Bits
		FL_ADDR,						//	FLASH Address bus 22 Bits
		FL_WE_N,						//	FLASH Write Enable
		FL_RST_N,						//	FLASH Reset
		FL_OE_N,						//	FLASH Output Enable
		FL_CE_N,						//	FLASH Chip Enable
		////////////////////	SRAM Interface		////////////////
		SRAM_DQ,						//	SRAM Data bus 16 Bits
		SRAM_ADDR,						//	SRAM Address bus 18 Bits
		SRAM_UB_N,						//	SRAM High-byte Data Mask 
		SRAM_LB_N,						//	SRAM Low-byte Data Mask 
		SRAM_WE_N,						//	SRAM Write Enable
		SRAM_CE_N,						//	SRAM Chip Enable
		SRAM_OE_N,						//	SRAM Output Enable
		////////////////////	ISP1362 Interface	////////////////
		OTG_DATA,						//	ISP1362 Data bus 16 Bits
		OTG_ADDR,						//	ISP1362 Address 2 Bits
		OTG_CS_N,						//	ISP1362 Chip Select
		OTG_RD_N,						//	ISP1362 Write
		OTG_WR_N,						//	ISP1362 Read
		OTG_RST_N,						//	ISP1362 Reset
		OTG_FSPEED,						//	USB Full Speed,	0 = Enable, Z = Disable
		OTG_LSPEED,						//	USB Low Speed, 	0 = Enable, Z = Disable
		OTG_INT0,						//	ISP1362 Interrupt 0
		OTG_INT1,						//	ISP1362 Interrupt 1
		OTG_DREQ0,						//	ISP1362 DMA Request 0
		OTG_DREQ1,						//	ISP1362 DMA Request 1
		OTG_DACK0_N,					//	ISP1362 DMA Acknowledge 0
		OTG_DACK1_N,					//	ISP1362 DMA Acknowledge 1
		////////////////////	LCD Module 16X2		////////////////
		LCD_ON,							//	LCD Power ON/OFF
		LCD_BLON,						//	LCD Back Light ON/OFF
		LCD_RW,							//	LCD Read/Write Select, 0 = Write, 1 = Read
		LCD_EN,							//	LCD Enable
		LCD_RS,							//	LCD Command/Data Select, 0 = Command, 1 = Data
		LCD_DATA,						//	LCD Data bus 8 bits
		////////////////////	SD_Card Interface	////////////////
		SD_DAT,							//	SD Card Data
		SD_DAT3,						//	SD Card Data 3
		SD_CMD,							//	SD Card Command Signal
		SD_CLK,							//	SD Card Clock
		////////////////////	USB JTAG link	////////////////////
		TDI,  							// CPLD -> FPGA (data in)
		TCK,  							// CPLD -> FPGA (clk)
		TCS,  							// CPLD -> FPGA (CS)
	    TDO,  							// FPGA -> CPLD (data out)
		////////////////////	I2C		////////////////////////////
		I2C_SDAT,						//	I2C Data
		I2C_SCLK,						//	I2C Clock
		////////////////////	PS2		////////////////////////////
		PS2_DAT,						//	PS2 Data
		PS2_CLK,						//	PS2 Clock
		////////////////////	VGA		////////////////////////////
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK,						//	VGA BLANK
		VGA_SYNC,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,  						//	VGA Blue[9:0]
		////////////	Ethernet Interface	////////////////////////
		ENET_DATA,						//	DM9000A DATA bus 16Bits
		ENET_CMD,						//	DM9000A Command/Data Select, 0 = Command, 1 = Data
		ENET_CS_N,						//	DM9000A Chip Select
		ENET_WR_N,						//	DM9000A Write
		ENET_RD_N,						//	DM9000A Read
		ENET_RST_N,						//	DM9000A Reset
		ENET_INT,						//	DM9000A Interrupt
		ENET_CLK,						//	DM9000A Clock 25 MHz
		////////////////	Audio CODEC		////////////////////////
		AUD_ADCLRCK,					//	Audio CODEC ADC LR Clock
		AUD_ADCDAT,						//	Audio CODEC ADC Data
		AUD_DACLRCK,					//	Audio CODEC DAC LR Clock
		AUD_DACDAT,						//	Audio CODEC DAC Data
		AUD_BCLK,						//	Audio CODEC Bit-Stream Clock
		AUD_XCK,						//	Audio CODEC Chip Clock
		////////////////	TV Decoder		////////////////////////
		TD_DATA,    					//	TV Decoder Data bus 8 bits
		TD_HS,							//	TV Decoder H_SYNC
		TD_VS,							//	TV Decoder V_SYNC
		TD_RESET,						//	TV Decoder Reset
		////////////////////	GPIO	////////////////////////////
		GPIO_0,							//	GPIO Connection 0
		GPIO_1							//	GPIO Connection 1
	);

////////////////////////	Clock Input	 	////////////////////////
input			CLOCK_27;				//	27 MHz
input			CLOCK_50;				//	50 MHz
input			EXT_CLOCK;				//	External Clock
////////////////////////	Push Button		////////////////////////
input	[3:0]	KEY;					//	Pushbutton[3:0]
////////////////////////	DPDT Switch		////////////////////////
input	[17:0]	SW;						//	Toggle Switch[17:0]
////////////////////////	7-SEG Dispaly	////////////////////////
output	[6:0]	HEX0;					//	Seven Segment Digit 0
output	[6:0]	HEX1;					//	Seven Segment Digit 1
output	[6:0]	HEX2;					//	Seven Segment Digit 2
output	[6:0]	HEX3;					//	Seven Segment Digit 3
output	[6:0]	HEX4;					//	Seven Segment Digit 4
output	[6:0]	HEX5;					//	Seven Segment Digit 5
output	[6:0]	HEX6;					//	Seven Segment Digit 6
output	[6:0]	HEX7;					//	Seven Segment Digit 7
////////////////////////////	LED		////////////////////////////
output	[8:0]	LEDG;					//	LED Green[8:0]
output	[17:0]	LEDR;					//	LED Red[17:0]
////////////////////////////	UART	////////////////////////////
output			UART_TXD;				//	UART Transmitter
input			UART_RXD;				//	UART Receiver
////////////////////////////	IRDA	////////////////////////////
output			IRDA_TXD;				//	IRDA Transmitter
input			IRDA_RXD;				//	IRDA Receiver
///////////////////////		SDRAM Interface	////////////////////////
inout	[15:0]	DRAM_DQ;				//	SDRAM Data bus 16 Bits
output	[11:0]	DRAM_ADDR;				//	SDRAM Address bus 12 Bits
output			DRAM_LDQM;				//	SDRAM Low-byte Data Mask 
output			DRAM_UDQM;				//	SDRAM High-byte Data Mask
output			DRAM_WE_N;				//	SDRAM Write Enable
output			DRAM_CAS_N;				//	SDRAM Column Address Strobe
output			DRAM_RAS_N;				//	SDRAM Row Address Strobe
output			DRAM_CS_N;				//	SDRAM Chip Select
output			DRAM_BA_0;				//	SDRAM Bank Address 0
output			DRAM_BA_1;				//	SDRAM Bank Address 0
output			DRAM_CLK;				//	SDRAM Clock
output			DRAM_CKE;				//	SDRAM Clock Enable
////////////////////////	Flash Interface	////////////////////////
inout	[7:0]	FL_DQ;					//	FLASH Data bus 8 Bits
output	[21:0]	FL_ADDR;				//	FLASH Address bus 22 Bits
output			FL_WE_N;				//	FLASH Write Enable
output			FL_RST_N;				//	FLASH Reset
output			FL_OE_N;				//	FLASH Output Enable
output			FL_CE_N;				//	FLASH Chip Enable
////////////////////////	SRAM Interface	////////////////////////
inout	[15:0]	SRAM_DQ;				//	SRAM Data bus 16 Bits
output	[17:0]	SRAM_ADDR;				//	SRAM Address bus 18 Bits
output			SRAM_UB_N;				//	SRAM High-byte Data Mask 
output			SRAM_LB_N;				//	SRAM Low-byte Data Mask 
output			SRAM_WE_N;				//	SRAM Write Enable
output			SRAM_CE_N;				//	SRAM Chip Enable
output			SRAM_OE_N;				//	SRAM Output Enable
////////////////////	ISP1362 Interface	////////////////////////
inout	[15:0]	OTG_DATA;				//	ISP1362 Data bus 16 Bits
output	[1:0]	OTG_ADDR;				//	ISP1362 Address 2 Bits
output			OTG_CS_N;				//	ISP1362 Chip Select
output			OTG_RD_N;				//	ISP1362 Write
output			OTG_WR_N;				//	ISP1362 Read
output			OTG_RST_N;				//	ISP1362 Reset
output			OTG_FSPEED;				//	USB Full Speed,	0 = Enable, Z = Disable
output			OTG_LSPEED;				//	USB Low Speed, 	0 = Enable, Z = Disable
input			OTG_INT0;				//	ISP1362 Interrupt 0
input			OTG_INT1;				//	ISP1362 Interrupt 1
input			OTG_DREQ0;				//	ISP1362 DMA Request 0
input			OTG_DREQ1;				//	ISP1362 DMA Request 1
output			OTG_DACK0_N;			//	ISP1362 DMA Acknowledge 0
output			OTG_DACK1_N;			//	ISP1362 DMA Acknowledge 1
////////////////////	LCD Module 16X2	////////////////////////////
inout	[7:0]	LCD_DATA;				//	LCD Data bus 8 bits
output			LCD_ON;					//	LCD Power ON/OFF
output			LCD_BLON;				//	LCD Back Light ON/OFF
output			LCD_RW;					//	LCD Read/Write Select, 0 = Write, 1 = Read
output			LCD_EN;					//	LCD Enable
output			LCD_RS;					//	LCD Command/Data Select, 0 = Command, 1 = Data
////////////////////	SD Card Interface	////////////////////////
inout			SD_DAT;					//	SD Card Data
inout			SD_DAT3;				//	SD Card Data 3
inout			SD_CMD;					//	SD Card Command Signal
output			SD_CLK;					//	SD Card Clock
////////////////////////	I2C		////////////////////////////////
inout			I2C_SDAT;				//	I2C Data
output			I2C_SCLK;				//	I2C Clock
////////////////////////	PS2		////////////////////////////////
input		 	PS2_DAT;				//	PS2 Data
input			PS2_CLK;				//	PS2 Clock
////////////////////	USB JTAG link	////////////////////////////
input  			TDI;					// CPLD -> FPGA (data in)
input  			TCK;					// CPLD -> FPGA (clk)
input  			TCS;					// CPLD -> FPGA (CS)
output 			TDO;					// FPGA -> CPLD (data out)
////////////////////////	VGA			////////////////////////////
output			VGA_CLK;   				//	VGA Clock
output			VGA_HS;					//	VGA H_SYNC
output			VGA_VS;					//	VGA V_SYNC
output			VGA_BLANK;				//	VGA BLANK
output			VGA_SYNC;				//	VGA SYNC
output	[9:0]	VGA_R;   				//	VGA Red[9:0]
output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
////////////////	Ethernet Interface	////////////////////////////
inout	[15:0]	ENET_DATA;				//	DM9000A DATA bus 16Bits
output			ENET_CMD;				//	DM9000A Command/Data Select, 0 = Command, 1 = Data
output			ENET_CS_N;				//	DM9000A Chip Select
output			ENET_WR_N;				//	DM9000A Write
output			ENET_RD_N;				//	DM9000A Read
output			ENET_RST_N;				//	DM9000A Reset
input			ENET_INT;				//	DM9000A Interrupt
output			ENET_CLK;				//	DM9000A Clock 25 MHz
////////////////////	Audio CODEC		////////////////////////////
inout			AUD_ADCLRCK;			//	Audio CODEC ADC LR Clock
input			AUD_ADCDAT;				//	Audio CODEC ADC Data
inout			AUD_DACLRCK;			//	Audio CODEC DAC LR Clock
output			AUD_DACDAT;				//	Audio CODEC DAC Data
inout			AUD_BCLK;				//	Audio CODEC Bit-Stream Clock
output			AUD_XCK;				//	Audio CODEC Chip Clock
////////////////////	TV Devoder		////////////////////////////
input	[7:0]	TD_DATA;    			//	TV Decoder Data bus 8 bits
input			TD_HS;					//	TV Decoder H_SYNC
input			TD_VS;					//	TV Decoder V_SYNC
output			TD_RESET;				//	TV Decoder Reset
////////////////////////	GPIO	////////////////////////////////
inout	[35:0]	GPIO_0;					//	GPIO Connection 0
inout	[35:0]	GPIO_1;					//	GPIO Connection 1

assign	LCD_ON		=	1'b1;
assign	LCD_BLON	=	1'b1;
assign	TD_RESET	=	1'b1;

//	All inout port turn to tri-state
assign	FL_DQ		=	8'hzz;
assign	SRAM_DQ		=	16'hzzzz;
assign	OTG_DATA	=	16'hzzzz;
assign	LCD_DATA	=	8'hzz;
assign	SD_DAT		=	1'bz;
assign	I2C_SDAT	=	1'bz;
assign	ENET_DATA	=	16'hzzzz;
assign	AUD_ADCLRCK	=	1'bz;
assign	AUD_DACLRCK	=	1'bz;
assign	AUD_BCLK	=	1'bz;

//	CCD
wire	[9:0]	CCD_DATA;
wire			CCD_SDAT;
wire			CCD_SCLK;
wire			CCD_FLASH;
wire			CCD_FVAL;
wire			CCD_LVAL;
wire			CCD_PIXCLK;
reg				CCD_MCLK;	//	CCD Master Clock

wire	[15:0]	Read_DATA1;
wire	[15:0]	Read_DATA2;
wire			VGA_CTRL_CLK;
wire			AUD_CTRL_CLK;
wire	[9:0]	mCCD_DATA;
wire			mCCD_DVAL;
wire			mCCD_DVAL_d;
wire	[10:0]	X_Cont;
wire	[10:0]	Y_Cont;
wire	[31:0]	Frame_Cont;
wire	[9:0]	mCCD_R;
wire	[9:0]	mCCD_G;
wire	[9:0]	mCCD_B;
wire			DLY_RST_0;
wire			DLY_RST_1;
wire			DLY_RST_2;
wire			Read;
reg		[9:0]	rCCD_DATA;
reg				rCCD_LVAL;
reg				rCCD_FVAL;
wire	[9:0]	sCCD_R;
wire	[9:0]	sCCD_G;
wire	[9:0]	sCCD_B;
wire			sCCD_DVAL;

//	For Sensor 1
assign	CCD_DATA[0]	=	GPIO_1[0];
assign	CCD_DATA[1]	=	GPIO_1[1];
assign	CCD_DATA[2]	=	GPIO_1[5];
assign	CCD_DATA[3]	=	GPIO_1[3];
assign	CCD_DATA[4]	=	GPIO_1[2];
assign	CCD_DATA[5]	=	GPIO_1[4];
assign	CCD_DATA[6]	=	GPIO_1[6];
assign	CCD_DATA[7]	=	GPIO_1[7];
assign	CCD_DATA[8]	=	GPIO_1[8];
assign	CCD_DATA[9]	=	GPIO_1[9];
assign	GPIO_1[11]	=	CCD_MCLK;
assign	GPIO_1[15]	=	CCD_SDAT;
assign	GPIO_1[14]	=	CCD_SCLK;
assign	CCD_FVAL	=	GPIO_1[13];
assign	CCD_LVAL	=	GPIO_1[12];
assign	CCD_PIXCLK	=	GPIO_1[10];
//	For Sensor 2
/*
assign	CCD_DATA[0]	=	GPIO_1[0+20];
assign	CCD_DATA[1]	=	GPIO_1[1+20];
assign	CCD_DATA[2]	=	GPIO_1[5+20];
assign	CCD_DATA[3]	=	GPIO_1[3+20];
assign	CCD_DATA[4]	=	GPIO_1[2+20];
assign	CCD_DATA[5]	=	GPIO_1[4+20];
assign	CCD_DATA[6]	=	GPIO_1[6+20];
assign	CCD_DATA[7]	=	GPIO_1[7+20];
assign	CCD_DATA[8]	=	GPIO_1[8+20];
assign	CCD_DATA[9]	=	GPIO_1[9+20];
assign	GPIO_1[11+20]	=	CCD_MCLK;
assign	GPIO_1[15+20]	=	CCD_SDAT;
assign	GPIO_1[14+20]	=	CCD_SCLK;
assign	CCD_FVAL	=	GPIO_1[13+20];
assign	CCD_LVAL	=	GPIO_1[12+20];
assign	CCD_PIXCLK	=	GPIO_1[10+20];
*/
//assign	LEDR[0]		=	mem_data;
assign	LEDR[9:0] = Xaddr_center;
assign	LEDG		=	Y_Cont;
assign	VGA_CTRL_CLK=	CCD_MCLK;
assign	VGA_CLK		=	~CCD_MCLK;

//------------------------------------------------------------------------------
//------------				user codes				---------------
//------------------------------------------------------------------------------
/////////////////////Definition BEGIN////////////////////////////////////
wire	RED_match, GREEN_match, BLUE_match, RED_GT_GREEN_match,RED_GT_BLUE_match;
wire	y_RED_match, y_GREEN_match, y_BLUE_match;
wire	g_RED_match, g_GREEN_match, g_BLUE_match;
wire	detect, R_detect, G_detect, Y_detect;
reg penup =0, red_present =0, green_present =0, yellow_present =0;
reg [9:0] red_paint = 10'h000;
reg [9:0] green_paint = 10'h3ff, blue_paint= 10'h000;
//VGA input RGB data
wire	[9:0]	VGA_data_iRed, VGA_data_iGreen, VGA_data_iBlue;//data from SDRAM
//reg	signed [9:0]	VGA_Y, VGA_U, VGA_V;//YUV
reg	[9:0]	VGA_iRed, VGA_iGreen, VGA_iBlue;//data into VGA
reg	[1:0] mem_data;//when detect, set this to one; otherwise, 0
reg	we;
wire	[1:0] readout;
reg	[16:0]	wr_addr, rd_addr;
wire	[9:0]		X_addr, Y_addr;
reg	flag;
reg	[9:0] xadd;
reg	[8:0] yadd;
reg	[1:0] state;
//center node
reg	[21:0] Xaddr_sum, Yaddr_sum;
reg	[9:0]	Xaddr_center, Xaddr_center_pre;
reg	[8:0]	Yaddr_center, Yaddr_center_pre;
reg	[9:0]	counter;
reg	draw;
reg	[1:0] color_sel;
wire	[9:0] Xaddr_draw;
wire	[8:0] Yaddr_draw;
wire	finished;
reg	[2:0] counter_addr1, counter_addr2;
wire back;

parameter init = 2'b00, init1 = 2'b01;

//original data coming from SDRAM
assign	VGA_data_iRed = Read_DATA2[9:0];
assign	VGA_data_iGreen = {Read_DATA1[14:10],Read_DATA2[14:10]};
assign	VGA_data_iBlue	= Read_DATA1[9:0];

//Red Color intensity checks
assign	RED_match=(VGA_data_iRed > 10'h100)? 1'b1:1'b0;  //Red > 128
assign	GREEN_match=(VGA_data_iGreen > 10'h080)? 1'b1:1'b0;	//Green > 128
assign	BLUE_match=(VGA_data_iBlue > 10'h080)? 1'b1:1'b0;		//Blue > 128

//Yellow Color intensity check
assign	y_RED_match=(VGA_data_iRed > 10'h100)? 1'b1:1'b0;  //Red > 128 100
assign	y_GREEN_match=(VGA_data_iGreen > 10'h100)? 1'b1:1'b0;	//Green > 128  140
assign	y_BLUE_match=(VGA_data_iBlue < 10'h080)? 1'b1:1'b0;		//Blue > 128  100

//Green color intensity check
assign	g_RED_match=(VGA_data_iRed < 10'd100)? 1'b1:1'b0;  //Red > 128 100
assign	g_GREEN_match=(VGA_data_iGreen > 10'd140)? 1'b1:1'b0;	//Green > 128  140
assign	g_BLUE_match=(VGA_data_iBlue < 10'h100)? 1'b1:1'b0;		//Blue > 128  100

//For RED Greater Then Green detection
assign	RED_GT_GREEN_match=(VGA_data_iRed > (VGA_data_iGreen + VGA_data_iGreen[9:1]))?1'b1:1'b0; //Red > 1.5(Green)
//For RED Greater Then Blue detection
assign	RED_GT_BLUE_match=(VGA_data_iRed > {VGA_data_iBlue[8:0],1'b0})?1'b1:1'b0;  //Red > 2(Blue)

//For Green Greater Then Red detection
assign	GREEN_GT_RED_match=(VGA_data_iGreen > {VGA_data_iRed[8:0],1'b0})?1'b1:1'b0; 
//For Green Greater Then Blue detection
assign	GREEN_GT_BLUE_match=(VGA_data_iGreen> {VGA_data_iBlue[8:0],1'b0})?1'b1:1'b0;  


// set match if R>128 and R> 2(G) and R > 2(B) and G < 128 and B < 128
assign	R_detect = RED_match & RED_GT_GREEN_match & RED_GT_BLUE_match & ~GREEN_match &  ~BLUE_match;
assign	Y_detect = y_RED_match & y_GREEN_match & y_BLUE_match;
assign	G_detect = g_RED_match & g_GREEN_match & g_BLUE_match;
assign   back = SW[17];



always@(posedge VGA_CTRL_CLK) begin

if(~KEY[0]) begin   //if KEY0 is pressed KEY0 becomes 0
	VGA_iRed <= 10'b0;
	VGA_iGreen <= 10'b0;
	VGA_iBlue <= 10'b0;
	counter <= 10'b0;
	Xaddr_sum <= 22'b0;
	Yaddr_sum <= 22'b0;
	Xaddr_center <= 10'b0;
	Yaddr_center <= 9'b0;
	Xaddr_center_pre <= 10'b0;
	Yaddr_center_pre <= 9'b0;
	counter_addr1 <= 2'b0;
	counter_addr2 <= 2'b0;
	//reset M4K
	if(~KEY[1]) begin   //if KEY0 is pressed and u press KEY1 also then u reinitialize the M$K addr to begin clearing the M4k
		xadd <= 10'b0;
		yadd <= 9'b0;
		flag <= 1'b0;
		state <= init;
	end
	else 					//when u release the KEY1 while KEY0 is stil presses u begin clearing the M4K so hold the KEY0 for some time
	case(state)		
		init: begin
        if (flag == 1'b0)	begin	
			 we <= 1'b1;		//set write enable for M4K
			 mem_data <= 2'b0;  
		    wr_addr <= {xadd[9:1], yadd[8:1]};	
		    state <= init1;
		  end
		  else begin
			 state <= init;
			 we <= 1'b0;
		    mem_data <= 2'b0;
		  end
		end
		init1:begin
		  flag <= 1'b1;
		  wr_addr <= {xadd[9:1], yadd[8:1]};
		  if(xadd < 10'd640 ) begin
		    xadd <= xadd + 10'b1;
		    state <= init1;
		  end
		  else if(xadd == 10'd640 && yadd < 9'd480) begin
		    xadd <= 10'b0;
		    yadd <= yadd + 9'b1;
		    state <= init1;
		  end
		  else begin
		    state <= init;
		  end
		end
	endcase
end

else begin      //if KEY0 is not pressed come here start executing
	//read from M4K
	rd_addr <= {X_addr[9:1],Y_addr[8:1]};  //get the address from the VGA controller and fed it to M4K
	if(readout == 2'b01) begin		//readout is the data read from M4K
		VGA_iRed <= 10'h3FF;
		VGA_iGreen <= 10'h0;
		VGA_iBlue <= 10'h0;
	end
	else if(readout == 2'b10) begin //Green color
		VGA_iRed <= 10'h0;
		VGA_iGreen <= 10'h3FF;
		VGA_iBlue <= 10'h0;
	end
	else if(readout == 2'b11) begin //Blue color
		VGA_iRed <= 10'h0;
		VGA_iGreen <= 10'h0;
		VGA_iBlue <= 10'h3FF;
	end
	else begin
	    if(back) begin
		//Display white background
		VGA_iRed <= 10'h3FF;
		VGA_iGreen <= 10'h3FF;
		VGA_iBlue <= 10'h3FF;
		end
		else begin
		
		//Display the original background for debugging only
		VGA_iRed <= VGA_data_iRed;
		VGA_iGreen <= VGA_data_iGreen;
		VGA_iBlue <= VGA_data_iBlue;
		end
	end
	
	if(Y_addr < 'd478) begin
		we <= 1'b0;
		if(R_detect) begin
			red_present <= 1;
			penup <=1;
		end
		if (G_detect)
			green_present <=1;
		if (Y_detect) begin
			yellow_present <=1;
		end
		
		if(Y_detect) begin//draw the color
			draw <= 1'b1;
			Xaddr_sum <= Xaddr_sum + X_addr;	
			Yaddr_sum <= Yaddr_sum + Y_addr;
			counter <= counter + 1;			
		end
		else	begin
			mem_data <= 2'b0;
		end
	end
	else if((X_addr <= 'd2) && (Y_addr == 'd478)) begin//calculate the center pixel and draw the pixel
		Xaddr_center <= Xaddr_sum/counter;
		Yaddr_center <= Yaddr_sum/counter;
	end 
	else if(Y_addr == 'd478) begin
		we <= draw;
		if((X_addr == 'd600) && we && (!penup)) begin
			Xaddr_center_pre <= Xaddr_center;
			Yaddr_center_pre <= Yaddr_center;
		end
		if(we) begin
			if((counter_addr1 != 3'b111) && (color_sel == 2'b00)) begin
				if(counter_addr2 != 3'b111) begin
				wr_addr <= {Xaddr_center[9:3],counter_addr1[1:0],Yaddr_center[8:3],counter_addr2[1:0]};
				counter_addr2 <= counter_addr2 + 1;
				end
				else begin
					counter_addr1 <= counter_addr1 + 1;
					counter_addr2 <= 3'b000;
				end
			end
			else if(!finished  && (!penup))
			wr_addr <= {Xaddr_draw[9:1],Yaddr_draw[8:1]};
		end
		else	wr_addr <= 17'b0;
		
		mem_data <= penup ? 2'b0: color_sel;     //if penup that is red detect is true just retain the mem data		
		//Color selection logic
		if ((Xaddr_center < 10'd120) && (Yaddr_center < 9'd150) && (|Xaddr_center)) begin
			color_sel <= 2'b01;  //RED paint
		end
		else if ((Xaddr_center < 10'd120) && (Yaddr_center > 9'd150)&& (Yaddr_center < 9'd240))begin
			color_sel <= 2'b10;  //GREEN paint
		end
		else if ((Xaddr_center < 10'd120) && (Yaddr_center > 9'd240)&& (Yaddr_center < 9'd330)) begin
			color_sel <= 2'b11;  //BLUE paint
		end
		else if ((Xaddr_center < 10'd120) && (Yaddr_center > 9'd330)) begin
			color_sel <= 2'b00;  //WHITE paint
		end
		else begin
			color_sel <= color_sel;   //retain the last chosen color
		end
	end
	else begin
		penup <=0;
		red_present <=0;
		green_present <=0;
		yellow_present <=0;
		draw <= 1'b0;
		Xaddr_sum <= 22'h0;
		Yaddr_sum <= 22'h0;
		counter <= 10'b0;
		//Xaddr_center <= 10'b0;
		//Yaddr_center <= 9'b0;
		we <= 1'b0;
		counter_addr1 <= 3'b000;
		counter_addr2 <= 3'b000;
	end
end
end


//M4K block interface
mem	pixel_addr (
	.clock (VGA_CTRL_CLK),
	.data ( mem_data ),
	.rdaddress ( rd_addr ),
	.wraddress ( wr_addr ),
	.wren ( we ),
	.q ( readout )
	);
	
gpu_func_draw_line line_drawing(
	.clk(VGA_CTRL_CLK),
   .reset(~KEY[0]),
   .start(we && (!penup) && (|Xaddr_center)),
   .finished(finished),
   .x1(Xaddr_center_pre),
   .y1(Yaddr_center_pre),
   .x2(Xaddr_center),
   .y2(Yaddr_center),
	.pos_x(Xaddr_draw),
   .pos_y(Yaddr_draw)
	);
//-----------------------	END	---------------------------

always@(posedge CLOCK_50)	CCD_MCLK	<=	~CCD_MCLK;

always@(posedge CCD_PIXCLK)
begin
	rCCD_DATA	<=	CCD_DATA;
	rCCD_LVAL	<=	CCD_LVAL;
	rCCD_FVAL	<=	CCD_FVAL;
end

VGA_Controller		u1	(	//	Host Side
							.oRequest(Read),
							.iRed(VGA_iRed),
							.iGreen(VGA_iGreen),
							.iBlue(VGA_iBlue),
							.iCursor_RGB_EN({penup && (|Xaddr_center),3'b0}),
							.iCursor_X(Xaddr_center),
							.iCursor_Y(Yaddr_center),
							.iCursor_R(10'd186),
							.iCursor_G(10'd85),
							.iCursor_B(10'd211),
							.H_Cont_out(X_addr),
							.V_Cont_out(Y_addr),
							//	VGA Side
							.oVGA_R(VGA_R),
							.oVGA_G(VGA_G),
							.oVGA_B(VGA_B),
							.oVGA_H_SYNC(VGA_HS),
							.oVGA_V_SYNC(VGA_VS),
							.oVGA_SYNC(VGA_SYNC),
							.oVGA_BLANK(VGA_BLANK),
							//	Control Signal
							.iCLK(VGA_CTRL_CLK),
							.iRST_N(DLY_RST_2)	);

Reset_Delay			u2	(	.iCLK(CLOCK_50),
							.iRST(KEY[0]),
							.oRST_0(DLY_RST_0),
							.oRST_1(DLY_RST_1),
							.oRST_2(DLY_RST_2)	);

CCD_Capture			u3	(	.oDATA(mCCD_DATA),
							.oDVAL(mCCD_DVAL),
							.oX_Cont(X_Cont),
							.oY_Cont(Y_Cont),
							.oFrame_Cont(Frame_Cont),
							.iDATA(rCCD_DATA),
							.iFVAL(rCCD_FVAL),
							.iLVAL(rCCD_LVAL),
							.iSTART(!KEY[3]),
							.iEND(!KEY[2]),
							.iCLK(CCD_PIXCLK),
							.iRST(DLY_RST_1)	);

RAW2RGB				u4	(	.oRed(mCCD_R),
							.oGreen(mCCD_G),
							.oBlue(mCCD_B),
							.oDVAL(mCCD_DVAL_d),
							.iX_Cont(X_Cont),
							.iY_Cont(Y_Cont),
							.iDATA(mCCD_DATA),
							.iDVAL(mCCD_DVAL),
							.iCLK(CCD_PIXCLK),
							.iRST(DLY_RST_1)	);

SEG7_LUT_8 			u5	(	.oSEG0(HEX0),.oSEG1(HEX1),
							.oSEG2(HEX2),.oSEG3(HEX3),
							.oSEG4(HEX4),.oSEG5(HEX5),
							.oSEG6(HEX6),.oSEG7(HEX7),
							.iDIG(Frame_Cont) );

Sdram_Control_4Port	u6	(	//	HOST Side
						    .REF_CLK(CLOCK_50),
						    .RESET_N(1'b1),
							//	FIFO Write Side 1
						    .WR1_DATA(	{sCCD_G[9:5],
										 sCCD_B[9:0]}),
							.WR1(sCCD_DVAL),
							.WR1_ADDR(0),
							.WR1_MAX_ADDR(640*512),
							.WR1_LENGTH(9'h100),
							.WR1_LOAD(!DLY_RST_0),
							.WR1_CLK(CCD_PIXCLK),
							//	FIFO Write Side 2
						    .WR2_DATA(	{sCCD_G[4:0],
										 sCCD_R[9:0]}),
							.WR2(sCCD_DVAL),
							.WR2_ADDR(22'h100000),
							.WR2_MAX_ADDR(22'h100000+640*512),
							.WR2_LENGTH(9'h100),
							.WR2_LOAD(!DLY_RST_0),
							.WR2_CLK(CCD_PIXCLK),
							//	FIFO Read Side 1
						    .RD1_DATA(Read_DATA1),
				        	.RD1(Read),
				        	.RD1_ADDR(640*16),
							.RD1_MAX_ADDR(640*496),
							.RD1_LENGTH(9'h100),
				        	.RD1_LOAD(!DLY_RST_0),
							.RD1_CLK(VGA_CTRL_CLK),
							//	FIFO Read Side 2
						    .RD2_DATA(Read_DATA2),
				        	.RD2(Read),
				        	.RD2_ADDR(22'h100000+640*16),
							.RD2_MAX_ADDR(22'h100000+640*496),
							.RD2_LENGTH(9'h100),
				        	.RD2_LOAD(!DLY_RST_0),
							.RD2_CLK(VGA_CTRL_CLK),
							//	SDRAM Side
						    .SA(DRAM_ADDR),
						    .BA({DRAM_BA_1,DRAM_BA_0}),
						    .CS_N(DRAM_CS_N),
						    .CKE(DRAM_CKE),
						    .RAS_N(DRAM_RAS_N),
				            .CAS_N(DRAM_CAS_N),
				            .WE_N(DRAM_WE_N),
						    .DQ(DRAM_DQ),
				            .DQM({DRAM_UDQM,DRAM_LDQM}),
							.SDR_CLK(DRAM_CLK)	);

I2C_CCD_Config 		u7	(	//	Host Side
							.iCLK(CLOCK_50),
							.iRST_N(KEY[1]),
							.iExposure(SW[15:0]),
							//	I2C Side
							.I2C_SCLK(CCD_SCLK),
							.I2C_SDAT(CCD_SDAT)	);

Mirror_Col			u8	(	//	Input Side
							.iCCD_R(mCCD_R),
							.iCCD_G(mCCD_G),
							.iCCD_B(mCCD_B),
							.iCCD_DVAL(mCCD_DVAL_d),
							.iCCD_PIXCLK(CCD_PIXCLK),
							.iRST_N(DLY_RST_1),
							//	Output Side
							.oCCD_R(sCCD_R),
							.oCCD_G(sCCD_G),
							.oCCD_B(sCCD_B),
							.oCCD_DVAL(sCCD_DVAL));

endmodule
						
