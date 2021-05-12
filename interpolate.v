`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    16:35:27 06/26/2019 
// Design Name: 
// Module Name:    interpolate 
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
module interpolate(
    output reg signed [15:0] pwm_remain,
    input signed [15:0] pwm_value,
    input Clk
    );




assign pwm_remain=pwm_value; 








endmodule






/*

dvider_1 divider_1 (
.clk(Clk),
.dividend(dividend[15:0]),//input
.divisor(divisor[3:0]),   // divider
.rfd(rfd), 					// enable data send
.quotient(quotient[15:0]),    //remain
.fractional(fractional[3:0])
);

// delay and out section
assign delay_pointer[9:0] = delay_pointerB[9:0]; // input
assign delay_write= delay_writeB;
assign delay_input =delay_feedback ; 
 assign pwm_valuetemp =pwm_value[15:0] ; 

reg signed [15:0] pwm_value1;
reg signed [15:0] pwm_value2 ;

reg signed [4:0] pwm_counterB ;   //interpolate counter

reg signed [15:0] dividend;
reg [3:0] divisor;
// wire rfd;
wire signed [15:0] quotient ;
wire [3:0] fractional;



//reg divisor[3:0]=4'b1111;

always @ (posedge Clk) begin    // will need to work on

case (counterB[3:0])

	1: if (!pwm_counterB) pwm_value1<=pwm_valuetemp ;  // add input
	2: pwm_counterB<=pwm_counterB+1;
	 


	4 : if ((rfd) && (pwm_counterB==4'b1111)) dividend <=pwm_value2-pwm_value1;  // enable write and pwm counter is full , write then wait 8 cycles
	 12 : pwm_remain<=pwm_value2-(quotient*pwm_counterB) ; 
  
	13 : if( pwm_counterB==4'b1000) pwm_value2<=pwm_value1;  // shift values




endcase
 
end

*/
