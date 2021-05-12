`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    09:20:28 01/06/2016 
// Design Name: 
// Module Name:    lcdtft 
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
module lcdtft ( input Clk, output reg SCK , output reg SDA, output reg  CS, output reg DC , output reg Reset,

input [7:0] tftdata  , input [6:0] tftPosition  
//input tftSerial, input serialClock, input serialEnabler

); // inputs


reg initialize; 
reg [2:0] trap; // mask counter
reg screenBlank;
reg [4:0] i;  // data counter 
reg [7:0] column;  // pixel for column
reg [7:0] row; // pixels for row
reg [7:0] data_step;  
reg [2:0] fontCounter;
reg [7:0] fontStore;
reg [8:1] font_buffer [1:6] ; // store font data 5X7 down , right
reg [1:0] fontSize; // 1 or 2
reg fontRead; 
reg fontWrite;
reg fontPass; 
reg [5:0] iLimit;  // set 16 bit write , 10 or 18
reg [6:0] font_step; 
reg [3:0] pixel_bit;
reg waitStep; 
//wire [9:0] spi_pointerB=spi_pointer+trap; 
reg [9:0] spi_pointerB;
reg [3:0] sck_counter ;
reg [8:0] datatemp;
reg [23:0] spiSlow; 
reg [9:0] delaysend; 
reg  [15:0] spi_send; // 16 bit value
reg [7:0] spicommands [0:512]; // spi memory first bit  is data flag
reg [7:0] spicommandsB [0:127] ; // tftdata incoming mem

reg [6:0] dataGrab; //grab location data 
reg writeEnableA;
reg [8:0] spi_pointerC; 
reg [8:0] spi_pointer;  // data pointer remember to change !!!
wire [6:0] spi_pointerout= tftPosition;   // write to pointer

//reg [15:0] tftSerialbufferB; 
//reg [15:0] tftSerialbuffer; // store incoming tftdata, 15 is parity
//reg [3:0] serialCount; // buffer counter

//wire [7:0] tftdata; //= tftSerialbuffer[7:0];
//wire [6:0] tftPosition; //=tftSerialbuffer [14:8];  // set output of buffer
//assign [15:0] tftSerialbuffer= {

/*
always @ (posedge serialClock ) begin  // input serial decoder
if (serialEnabler) begin   // problem here 
serialCount<=serialCount+1;  // up count
tftSerialbufferB[serialCount]<=tftSerial;
end
else begin serialCount<=0;tftSerialbuffer<=tftSerialbufferB; end // dump collected data and reset counter
end  
*/


initial
begin
$readmemh("spicommands.lst" ,spicommands,0,512);  // read memory from file // 500-600 screen data reference
$readmemh ("spicommandsB.lst",spicommandsB, 0,127); // initial menus 


dataGrab=0;
fontSize=2;
fontPass=0;
font_step=0;
column=0;
row=0;
trap=0;
pixel_bit=0; 
initialize=1;
spiSlow=64;
fontWrite<=0;
Reset=0; 
SDA=1;
iLimit=10;
waitStep=1;
delaysend=0;
data_step=1;
spi_pointer=0;
fontCounter=0;
fontStore=0;
i=10;
CS=1;
SCK=0;
sck_counter=0;
DC=0;
datatemp=0;
screenBlank<=1;
fontRead=1;
spi_pointerC=0;
end







always @ (posedge Clk) begin





//LED<=dataGrab;
// if (tftdata) spi_pointerout<=tftdata[5:0] +500;
  

//if (!trap)  begin    end 

// mask section

trap<=trap+1; 




if (trap && fontRead) begin
font_buffer[trap]<=spicommands[spi_pointerB];   // goes from 1-7  , font pixel data

end 


if (!fontRead )    begin 
spi_pointer<=((spicommandsB[dataGrab]-32) *5) + 18  ;  // font data pointer, grab initially
fontRead<=1;


end else spicommandsB[spi_pointerout]<=tftdata[7:0]; // separate writes


//spi_pointerC<=dataGrab+512; // get ascii from mem , too many drivers


if (fontRead)  spi_pointerB<=spi_pointer+trap;  

 // reset output

if (!spiSlow[17])  spiSlow<=spiSlow+1; // count to about a sec 

if (spiSlow[16] && Reset==0) Reset<=1; // initial reset , initilaize data is 1-6, 42 column , 43 row, 44 ram write, 16 bit data




// initialize section


if ( spiSlow[17] && initialize  ) begin      // after 1 sec starts full speed 

delaysend<=delaysend+1;
case (delaysend)  // short cycle

10'd127 : begin    spi_pointer<=spi_pointer+1;DC<=0; end  // count up spi_pointer 
10'd191 : begin if (spi_pointer==3) DC<=1;		end  // mode 5 
10'd255 : begin if (spi_pointer==9 && initialize ) begin  initialize<=0; data_step<=1; end  end  // finish initialize after pos 6 , 
//800 : begin spi_send[7:0] <=96; DC<=0; end // madctl 
//846 : begin spi_send[7:0] <= 96; DC<=1; end 
10'd1000 : begin  spi_send[7:0] <=font_buffer[1] ; i<=0;   end     // delay these a fair bit otherwise no go, not too much though,,   this sends out spi_pointer data 

endcase

 // reset startup 
end


// draw screen section

if  ((!initialize)  && (!waitStep)  )  begin


case (data_step) // long cycle

5'd1 : begin spi_send[7:0] <= 42 ; i<=0;DC<=0;iLimit<=10;  end  // enable column  
5'd2 : begin spi_send[7:0] <= 0 ; i<=0 ;DC<=1; end  
5'd3 : begin  spi_send[7:0] <= column ; i<=0; end // column write
5'd4 : begin  spi_send[7:0] <= 0 ; i<=0; end
5'd5 : begin  spi_send[7:0] <= column+9 ; i<=0; end 
5'd6 : begin  spi_send[7:0] <= 43 ; i<=0;DC<=0; end  // set row
5'd7 : begin  spi_send[7:0] <= 0 ; i<=0;DC<=1; end 
5'd8 : begin  spi_send[7:0] <= row ; i<=0; end  // row write 
5'd9 : begin spi_send[7:0] <=0 ; i<=0; end 
5'd10 : begin spi_send[7:0] <=row+13  ; i<=0; end 
5'd11 : begin  spi_send[7:0] <= 44 ; i<=0; DC<=0; 							end // ram write
5'd12 :begin
 if (screenBlank)  begin
i<=18;
iLimit<=18;
 DC<=1;
CS<=1;
pixel_bit<=0;
font_step<=0;
spi_pointer<=20; 
if (column!=120) column<=column+10; else begin  column <=0; row<=row+14 ; end 
data_step<=13;
end
else data_step<=14;
end

5'd13:begin
if (row==154) begin row<=0; screenBlank<=0; end 


data_step<=16;
end


5'd14 : begin 


i<=18;
iLimit<=18;
 DC<=1;
CS<=1;
font_step<=0;
fontRead<=0;
 //1 

pixel_bit<=0;
//trap<=0; // very important or it wont work 
 if (column!=108) column<=column+12; else begin  column <=0; row<=row+16 ; end  // count up coloumns after reading all font bytes
   // count up rows
 data_step<= data_step+1;
 dataGrab<=dataGrab+1; // proceed after writes ,  2 
end
5'd15 : begin
if (row==160) begin row<=0;dataGrab<=0; end 

 data_step<= data_step+1;
//LED<=spi_pointerC[9:2];
end

5'd16 : data_step<= data_step+1;
5'd17 :fontWrite<=1;


endcase
end 


fontStore [7:0]  <= font_buffer[(font_step>>1)+1];  // store temp for fonts 

if ( (data_step[0]) && fontWrite ) begin  data_step<=data_step+1;  end
if ((font_step>>1)==5 ) begin font_step<=0; pixel_bit<=pixel_bit+1; end   // double size 
if ((pixel_bit>>1)==7) begin  pixel_bit<=0;data_step<=0; fontWrite<=0;fontPass<=0;  end  

if (!fontStore[pixel_bit>>1] && fontWrite   ) begin  spi_send[15:0] <=0; fontPass<=1;  end  // finish here
if  (fontStore[pixel_bit>>1] && fontWrite ) begin spi_send[15:11] <= font_step<<1 ; spi_send[10:5]<=dataGrab ;spi_send[4:0]<=pixel_bit*3;  fontPass<=1; end // font colour info
if (fontPass && CS ) i<=0;  

// LED<=pixel_bit; 


// data send section , send 1 byte , dc controlled by bit [8] , spi_send is 8 bit data, starts with write allow, set i to 0 for start

//if (!i) begin // starts on i = 0


if (!i) begin CS<=0;i<=1; waitStep<=1;   end // go low at 0 , set dc as well , disable bitstep
//if (!i && waitStep==1)begin    end // count up one cycle 


if (i!=iLimit && i ) sck_counter<=sck_counter+1;  // 4 bit counter for clock 
if (!sck_counter  && (i!=iLimit) && i  ) begin i<=i+1;  SDA<=0;  end   // go low and progress for 1 cycle 

SCK<=sck_counter[3]; // clock signal

if ((sck_counter==7)  &&  (i!=iLimit) ) begin  SDA<= spi_send[iLimit-1-i];   end // send data MSB first


if (i==iLimit && (CS==0)) begin CS<=1; SDA<=0; sck_counter<=0;   delaysend<=0; data_step<=data_step+1; font_step<=font_step+1; fontPass<=0;end  // go high at end 

if (i==iLimit && CS==1) begin delaysend <=delaysend+1;  end 

if (i==iLimit  && delaysend[2] && !initialize ) begin    waitStep<=0;  end 



end // clockloop


endmodule
