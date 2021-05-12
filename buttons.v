`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    09:06:08 01/14/2016 
// Design Name: 
// Module Name:    buttons 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module buttons(
    input Clk,
	 output reg [4:0] BUTTONREAD, // button detect
    input [4:0] BUTTONWRITER,    // led control per bit
    output reg  [4:0] BUTTONLOW,
    inout  [3:0] BUTTONHIGH
    );
	 
	 

//reg [3:0] button_hightemp;

reg [3:0] button_highread; // input buffer for buttonhigh
reg [4:0] buttonOut; //store button values	0-19


reg low_enable; // enable buttonlow
reg [15:0] button_counter; // drive read and led write
reg [2:0] lowCounter;// keep track of 0-4 count 
reg [9:0] button_counterB;  //50khz counter
//reg [4:0] button_low_b; 
reg [3:0] high_store;
//reg [4:0] low_store;
reg  [3:0] button_enable;
wire [3:0] button_set =button_enable [3:0]; // needs to be a wire ? 
reg [1:0] button_highselect; //buttonhigh select pin 
assign BUTTONHIGH[0]=(button_set[0]) ? high_store[0] : 1'bz ;   // assing value or z , working good
assign BUTTONHIGH[1]=(button_set[1]) ? high_store[1] : 1'bz ;   // assing value or z , working good
assign BUTTONHIGH[2]=(button_set[2]) ? high_store[2] : 1'bz ;   // assing value or z , working good
assign BUTTONHIGH[3]=(button_set[3]) ? high_store[3] : 1'bz ;   // assing value or z , working good


always @ (posedge Clk) begin// need to kill scan if button held

 	
	
	
	
button_counterB<=button_counterB+1;  // fast counter ,stop while button is pressed 
if (!button_counterB  ) button_counter<=button_counter+1; // counter divided 10 bit , slow counter	
	
if (button_counter[4:3]==BUTTONWRITER[3:2])   high_store[BUTTONWRITER[1:0]]<=1 ; else high_store[3:0]<=0; 	// assign set value to leds , works good , fast ok as no effect on io
	
	//button_enable<=0;
	//if ((buttonOut!=5'b11111)&& buttonOut && (buttonOut!=button_temp))  begin 
	//BUTTONREAD[4:0]<=buttonOut[4:0]; button_temp<=buttonOut;buttonOut<=0;
	

	//if (button_counter[10] && button_delay) button_delay<=0; // about 25hz
	

case (button_counterB[9:7]) // fast loop 
	//1 : if  (buttonOut!=button_counter[4:1])  BUTTONLOW[4:0]<=~(1<<button_counter[4:3]); else BUTTONLOW[4:0]<=5'b11111;  //1011 etc slow count , needs to ignore if already set 
	

	
	2 :  begin  button_enable<=~(1<<button_counter[2:1]); BUTTONLOW[4:0]<=~(1<<button_counter[4:3]);end  //1101 etc fast count  , select pin enabled for reading , overules io write
  // 2 :  begin  button_enable<=0; BUTTONLOW[4:0]<=~(1<<button_counter[4:3]);end  //1101 etc fast count  , select pin enabled for reading , overules io write,this didnt work atm



	//3 : if (BUTTONHIGH[button_counter[2:1]]) begin BUTTONREAD[4:0] <=	button_counter[2:1]+(button_counter[4:3]*4)+1;  low_enable<=1;    end else 
	//begin  BUTTONREAD[4:0] <=0; low_enable<=0;  end  // really needs to stop here and retest until released , but it needs to loop
	
//	3 : if (BUTTONHIGH[3:0]) begin BUTTONREAD[4:0] <=	(button_counter[4:3]*4)+1;  low_enable<=1;    end else 
	//begin  BUTTONREAD[4:0] <=0; low_enable<=0;  end  // really needs to stop here and retest until released , but it needs to loop

	3 : if (BUTTONHIGH[button_counter[2:1]]) begin BUTTONREAD[4:0] <=	button_counter[2:1]+(button_counter[4:3]*4)+1;  low_enable<=1;    end else begin  BUTTONREAD[4:0] <=0; low_enable<=0;  end
	
	
	
endcase

end
	
endmodule
