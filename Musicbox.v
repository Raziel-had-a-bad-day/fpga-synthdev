`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    13:27:35 09/18/2015 
// Design Name: 
// Module Name:    Musicbox
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
module Musicbox ( input wire Clk,   output PWM, output PWMb, output  reg triggerA ,output reg  triggerB, output reg pwmLFO, output reg pwmLFOB,
input wire encoderA, input wire encoderB, input  inMidiA, input signed [15:0] pwm_remain ,
inout [3:0] BUTTONHIGH, output [4:0] BUTTONLOW, input wire UART1, input wire UART2,


output reg CS1 , output  SCK1, output reg Din, input Dout, output  signed [15:0] pwm_value  ,
output reg [7:0] LED,
 output  Reset , output  SCK, output  SDA, output  CS, output  DC, output reg midiClock, output reg midiMirror ,output reg TX,
 output UART1A,  output UART2A

); // keep ins/ outs separate from each other , also wire only one test
 
ringcounter ringcounter (
  .clk(Clk), // input clk
  .q(counterB[11:0]) // output [11 : 0] q

);










buttons buttons (
	.Clk(Clk),
	.BUTTONREAD(BUTTONREADA[4:0]),
	.BUTTONWRITER(BUTTONWRITEA[4:0]),  // problem crash if connected to a reg
	.BUTTONLOW(BUTTONLOW[4:0]),
	.BUTTONHIGH(BUTTONHIGH[3:0])
	);


wire [4:0] BUTTONREADA;
reg [4:0] BUTTONWRITEA ;//= buttonHolder[4:0]-2; 
//wire [4:0] BUTTONWRITEA=13; 
reg button_detect2;
reg [23:0]button_detect3;

reg [13:0] ledBlink ; // blinker
assign UART1A=UART1;
assign UART2A=UART2;

reg [4:0] buttonHoldout; 
reg [15:0] buttonHolder; 
reg [3:0] buttonCountup;  // smple counter for led on off
wire  button_detect= (BUTTONREADA) ?  1 :0 ; // detect button press detect

always @ (posedge Clk) begin

LED[0]<=ledBlink[13];

if (button_detect) button_detect2<=1; else button_detect2<=0;
if (button_detect2) button_detect3<=button_detect3+1; else button_detect3<=0; // delay counter
 



end


always @ (posedge counterB[11]) begin
ledBlink[13:0]<=ledBlink[13:0] +1;

end





always @ (posedge button_detect3[23] ) begin  // works puts in a delay ,good for filtering


buttonHolder[4:0] <=BUTTONREADA [4:0];  // sending straight for now /need to loop 




//if (buttonHolder[BUTTONREADA]) buttonHolder[BUTTONREADA] <=0; else  buttonHolder[BUTTONREADA] <=1;
//if ((buttonHolder[4:0] !=BUTTONREADA [4:0]) && (BUTTONREADA [4:0]))    buttonHolder[4:0] <=BUTTONREADA [4:0] ; else buttonHolder[4:0]<=0; 


end

always @ (posedge Clk) begin
buttonCountup<=buttonCountup+1  ;
if (buttonCountup[3:0]==buttonHolder[3:0] ) buttonHoldout[buttonCountup]<= !buttonHoldout[buttonCountup]; 
BUTTONWRITEA [4:0] <=buttonHolder[4:0]-2 ; //  buttonHoldout [4:0];



end













/*
delay_ram delay_ram (
  .clka(Clk), // input clka
  .wea(delay_write), // input [0 : 0] wea
  .addra(delay_pointer [9:0]   ), // input [3 : 0] addra
  .clkb(Clk), // input clkb
  .addrb(addrb[9:0]), // input [9 : 0] addrb
  .dina(delay_input [15:0] ), // input [16 : 0] dina
  .doutb(delay_output[15:0]) // output [16 : 0] douta
);
*/
lcdtft lcd1 (.Clk(Clk),
.Reset(Reset),
.SCK(SCK),
.SDA(SDA),
.CS(CS),
.DC(DC),
//.serialEnabler(serialEnablerB),
.tftdata(tftdataout[7:0]),
//.tftSerial(tftSerialB),
//.serialClock(serialClockB)
.tftPosition(tftPositionout[6:0])
); // start up second module


//assign divisor=divideConstant; 
//reg [2:0] divideConstant ;
//wire counterB8 = counterB[8] ; 
wire [11:0] counterB;
wire [6:0] tftPositionout;  // position of character for tft
wire [7:0] tftdataout ;			// character for tft

//reg [7:0] bcd_mask; 
//reg [31:0] bcd_maskA; // 4 * 8 bit characters sent every time 
//reg [6:0] bcd_maskstart; // begin of 4 characters
//reg bcdNumber=1; // enable if sending numbers

wire [3:0] bcd_mask= bcd_storeB>>(counterB[1:0]*4);  // showing 1 bit extra


assign tftdataout=48+bcd_mask  ; // send out adsr count , only when counterB[3:0] is true 
assign tftPositionout= (21+(note_sequenceC[3:1]*10) +(3-counterB[1:0]));   // sync position 0-3 digits
//assign tftPositionout= (21+(counterB[6:4]*10) +(3-counterB[1:0]));   // sync position 0-3 digits



/*
assign tftdataout=bcd_mask+(bcdNumber*48)   ; // send out adsr count, add 48 for numbers 
assign tftPositionout=bcd_maskstart+ (3-counterB[3:2]);   // write 4 digits from bcd_maskstart
always @ (posedge Clk) begin  // tft lcd out sorting 

if (!bcdNumber) begin 
 bcd_mask[7:0]<=(bcd_maskA>>(counterB[3:2]*8)); // read out 8 bit values and add 48 if a number 
end else 
begin
bcd_mask[3:0] <= (bcd_storeB>>(counterB[3:2]*4));
end
case (counterB[6:4])
0 : begin bcd_maskstart<=22;bcdNumber<=1;  end 
//1 : begin bcd_maskstart<=30; bcd_maskA<=32'h74656368; bcdNumber<=0; end
 endcase
end
*/



//assign tftPositionout=30+(3-counterB[1:0]);   // sync position 0-3 digits
/*
assign tftSerialB=tftSerialC;
reg tftSerialC; 
reg [4:0] tftSerialcount;
assign serialEnablerB=tftSerialcount[4];
always @ (posedge Clk ) begin // leave gaps for a while
if (!serialEnablerB && !counterB[11:10] ) tftBufferB<=tftBuffer; 
tftSerialcount<=tftSerialcount+1;
tftSerialC<=tftBufferB[tftSerialcount[3:0]];
end
assign serialClockB=Clk; // needs to flip earlier 
//reg [15:0] tftBuffer; 
reg [15:0] tftBufferB;  
wire [15:0] tftBuffer={1,tftPositionout[6:0],tftdataout[7:0] } ; // string position
*/


wire  [1:0] encoder = {encoderA,encoderB};// rotary encode input reg
reg [7:0] encoderout ; // encoder output value store 


//reg pwmHalf;
reg pwmstart =0 ; // pwm out reg
reg pwmstartB=0; 
reg halfcycle; // dont reset modulate counter until enabled , stops counting until true

 

reg [7:0] counter; // normal counter for pwm , doesnt change length
//reg [11:0] counterB; // lfo counter


reg [7:0] arraywave_c [0:62];  // wave array + note divider ,voltages
reg [15:0] arraywave_b [0:511];  // wave array + note divider , sine values, 16 bit halfwave 512 samples for half wave , using LUTS atm 
reg [7:0] arraywave [0:31]; // note freq values  

reg [0:0] changedpin;
//wire [4:0] modulateP = modulate [4:0] + 1;  
//wire [7:0] tftdata;




assign SCK1=pot_counter[0]; // clk for analogue input spi running at 50m/1024 


//assign triggerA =   |modulate[(midiByte3>>2)]   |   ~(!modulate) ;
//if (!modulate) triggerA<=0; ; // note freq output 0 or 1 when modulate at midibyte/
 


reg [17:0] Note;  // actual note 0-11 
reg [17:0] NoteB; // note 2

//reg [6:0] Octave; // octave of the noteinitial 
reg  tempCounter;  // enable freq process counter

// midi section
//reg [2:0] divisorA=3;
reg inMidi;
reg [3:0] note_oncounter; // count note ons 
reg [7:0] midiTemp; // temp data storage for serial in
reg [10:0] midiDivideA ;  // divide by 10
reg [7:0] midiStatus ; // midi sorting, 0=status, 1=data1 , 2=data2
reg [6:0] midibyte_1; // data byte 1 ,pitch
reg [6:0] midibyte_1B;//temp midi in note
reg [6:0] midibyte_2; // data byte 2 ,velocity
reg [6:0] midibyte_2B; //temp vel in midi
reg [6:0] midibyte_1C; // note temp for sequence
reg [6:0] midibyte_2C; // vel temp from sequencer

reg midiplay_note; // trigger note from midi in 
reg [13:0] midiByte3;//note off pitch value A
reg [13:0] midiByte4;//note off pitch value B
reg [15:0] midiByte5; // temp holder for display
reg [6:0] midi_fifo  [0:15]; // midi ring memory , note 
reg [6:0] midi_fifoB [0:15]; // midi ring memory ,  velocity 

reg [3:0] midiPointerA; // fifo read
reg [3:0] midiPointerB; //fifo pointers , write
reg midiFlip;
reg  [1:0] internalSeq; // enable reading from internal  sequencer , only when note is scheduled 
reg  [9:0] internal_note;  // int seq note
reg [9:0] internal_noteA; 
reg [6:0] internal_velocity;  // int seq velocity

//reg [6:0] midiRing[0:15] ; // ring buffer for incoming data 7 bits 5x2 
reg [3:0] countup; // midi ring counter
reg [5:0] countupB;// midi ring playback
reg [5:0]  arrayPointA; // mem pointer for osc vref
//reg [4:0] arrayPointB;  // mem note pointer from  incoming midi
reg [7:0] read_byte; // temp read for sorting
reg [1:0] read_byteA; // written flag 
reg play_note; // enable sound playback


reg finishRead; 
reg midiBaud;  // midi clock signal 31250hz 
reg [2:0] midi_start; //  midi start , 1 bit start detected, 2 while reading
reg [4:0] bitStep; //eediot
reg [1:0] midiType;  // midi data type counter

reg [2:0] byteCounter;// count read bytes 2 values for now
reg [17:0] note_resetA; 
reg [17:0] note_resetB ;  //Note reset storage holder  2 actual notes  18 is empty flag 
//wire [18:0] note_reset1=Note+1;
//wire note_resetbitB=note_reset1[18]; 

reg [17:0] note_resetC ;  //Note reset storage holder  2 actual notes  18 is empty flag 






//assign inMidi=inMidiA;



reg [15:0] pot_counter;  // spi analouge in counter 0-33 then reset at 37  
reg [2:0] pot_counterB; // count up 8 inputs
reg [7:0] pot_store [0:7] ;  // storage for pot values 10 bit / 8 pots 
reg [9:0] pot_temp; // temp storage for pot store 
reg [9:0] pot_tempB; 
reg [9:0] selectPot; // store some pot value from pot_store
reg [15:0] bcd_store;  // bcd value for display , 4*4 bits 
reg [15:0] bcd_storeB; //bcd value out 
reg [3:0] bcd_temp;
reg [3:0] bcd_counter; // bcd shift up counter
reg [3:0] n; //loop var
reg [15:0] arraytempo [0:190]; // block ram for tempo values 16 bit  
reg [14:0] tempo_counter; // tempo counter timer
//wire [16:0] tempo_counter0=tempo_counter+1; // overflow adder
//wire tempoReset0=tempo_counter0[16];  // detect overflow
reg [15:0] tempo_calc ; // calculated tempo out 
reg [7:0] tempo;  // tempo variable ,default 120  at 1/4 value , 240ms(250bpm)  to 1000ms(60bpm) 


reg [12:0] tempo_counterB; // clock counter

reg [3:0] note_sequence; // notesteps for sequencer
reg [3:0] note_sequenceB; 
reg [3:0] note_sequenceC; //for tft
reg [11:0] bcd_reader; // store any value up to 12 bits
reg [17:0] one_bit; // accumulator 12 bit in
reg [17:0] one_bitB; 
//reg [3:0] displayStep; // drive tft display position lines

//reg [7:0] LED =0; 
reg [17:0] freqCounter [0:127]; // freq lut 
reg [15:0] modulate_array[0:127] ; // modulate adder values x 2048 per midi note
reg signed [7:0] adsr_list [0:255]; //lut for adsr ROM  



initial
begin
// messageA="Hello World!";
$readmemh("sine.lst" ,arraywave_b,0,511);  // read memory from file , sine wave
$readmemh("arraywave_c.mif" ,arraywave_c,0,31);  // read memory from file modified
$readmemh("tempo.lst",arraytempo,0,190); // read 14bit tempo values (divide by 10 for correct value) 
$readmemh("arraywave.mif" ,arraywave,0,31);  // read memory from file, note frew
$readmemh("freqcounter.lst" ,freqCounter,0,127);  // read memory from file, note frew
$readmemh("modulate_array.lst" ,modulate_array,0,127);  
$readmemh("ADSR.lst" ,adsr_list, 0, 255);   // put reg before this 
//divideConstant=3;
modulate_conTemp=1;
modulate_conTempB=1;
pwmLFOB=0;
pwmLFO=0;
triggerB=0;
byteCounter=0;
midiDivideA=0;
midiFlip=1;
midiTemp=0;
read_byte=0;
delay_pointerA<=511;
delay_pointerB<=491;
midiType=0;

Note=0;    // simulator has problems if any of these are x value !!!!!!!
note_oncounter=1;
midiByte3=0;
midiByte4=0; 
midi_start=0;
midiBaud=1;
midiPointerA=0;
midiPointerB=0;
CS1=1;
pot_counter=0;
pot_counterB=0; 
pot_temp=0;
pot_tempB=0; 
selectPot=0; 
bcd_store=0;
bcd_storeB=0;
bcd_counter=0;
internalSeq=0;
tempo=120; 
internal_note=64;
internal_velocity=0;  // eediot
play_note=0; 
tempo_counterB=1595; 

bitStep=0;
changedpin=0;
counter=0;

encoderout=1;
lfoShape<=1;
lfoShapeB<=11'b11111111111;
adsrCounter=1;
adsrCounterB=1; 
adsrTrigger=0;
adsrStep=0;
sustain=32000;
ADSR1= 32'b01111111000000011111111100000001 ;   
adsrTriggerB=0;
adsrStepB=0;
sustainB=32000;
ADSR2= 32'b01111111000000011111111100000001 ;   
buttonHolder=0; 




end




// encoder section


always@ ( posedge encoderA) begin  // encoder in
changedpin=1 ;  
if (encoder[0]==encoder[1] ^ changedpin)   encoderout<=encoderout+1'b1;  else  encoderout<=encoderout-1'b1 ;changedpin=0  ;
end


// Midi section

always @( posedge Clk ) begin   // Midi data in , works ok 



 
//if (read_byteA==1) play_note<=0; // disable play_note once read_byte is set 
inMidi<=inMidiA; // assign input value


if (midiDivideA==799) begin midiBaud<=~midiBaud; midiDivideA<=0; end else  midiDivideA<=midiDivideA+1; // count to 800

if ((!inMidi) && (!bitStep)) begin midiDivideA<=0; midiBaud<=0; bitStep<=1;  byteCounter[1:0] <=2'b00 ; end  // start reading anytime , reset time counter

if ((!midiBaud) && bitStep[0] && bitStep!=22 ) begin bitStep<=bitStep+1; midiFlip <=~midiFlip; end // flip on on odd and low to even

if (midiBaud && bitStep[0] && bitStep!=3 && bitStep!=21 && midiFlip && midiDivideA==400   )begin  midiTemp[((bitStep-5)/2)]<=inMidi; midiFlip<=0; end // write inmidi to miditemp

if ( midiBaud && !bitStep[0] && bitStep)  begin bitStep<=bitStep+1; end  // flip on even and high to odd

if ( bitStep==21 && midiFlip ) begin // end of reading one byte 

if (midi_start) midi_start<=midi_start+1; // only once per byte
midiFlip<=0; 

if ((!midiTemp[7]) && (!midi_start)) begin midi_start<=2;    end // running status  
if (midiTemp[7:4]==4'b1111  && midi_start==0)  begin bitStep<=0; read_byte<=0; end     else  read_byte <= midiTemp;  // ignore midi timing pulses , read miditemp into read_byte 

end   // wrtie to ram ,reset to next bitstep  

if (bitStep==22) begin    // BYTE read loop 

midiFlip<=1;    // end of fetching data ,everything under this till the end of clk




if ((read_byte==144  |  read_byte==128) && (!midi_start))     begin    // no midi_start without this 

midiStatus <= read_byte;

midi_start<=1; 
  // read status byte Note On  only if at least one note is off, also start counter for byte reads , always counting , jump to start ,,step1
 end

 // if (read_byte && (!read_byte[6]) && (!midi_start)  ) begin  midibyte_1[6:0]<=read_byte[6:0]+5 ;  midi_start<=3; bitStep<=0; end // midi running status used if no status byte is sent,problems!

 if (read_byte   && (midi_start==2)   )   begin //midibyte_1[6:0]<=read_byte[6:0]+5 ; 

midibyte_1B<=read_byte[6:0]+5; // temp note
midiplay_note<=0; // stop just in case missed , for new values
 midi_start<=3; end //starts storing too early
 // read note value run cycle several times if needed,, step 2
 
 if ((midi_start==4)  ) begin 
	  
	midibyte_2B<=read_byte[6:0]; // temp vel
	
	midiplay_note<=1;
	
	midi_start <=0;  // if on and noteON  read note_velocity ,,step 3



end
bitStep<=0; // always end 

end  //  bitstep 22 new note end old notes  flip flop note locations

// disable for now 

if ((noteFlag[0])  && (!play_note) && (!midiplay_note))  begin noteFlag[1]<=1; midibyte_1C<=(48+(internal_note[6:2]));  midibyte_2C<=internal_velocity; play_note<=1;  // one cycle ,,may not work note keeps changing before not off, sequencer

end 
if (!play_note && midiplay_note && !noteFlag[0]) begin    midibyte_1C<=midibyte_1B; midibyte_2C<=midibyte_2B; midiplay_note<=0;play_note<=1; end // write temp from seq or midi ,midi
//else noteFlag[1]<=0; // note off issue 
if (play_note && noteFlag[1]) noteFlag[1]<=0; // reset after playing note


if (fifo_write && !fifo_read ) begin midi_fifo[countupB[3:0]]<=midibyte_1C; midi_fifoB[countupB[3:0]]<=midibyte_2C; fifo_write<=0;play_note <=0; // reset play_note
countupB<=0;countup[3:0]<=countupB[3:0]; 
fifo_read<=1;  // max 4 cycles latency 
 end  // trigger write to fifo

if (read_byteA==1) fifo_read<=0; // turn off fifo_read 

if (play_note && !fifo_write) begin // max 4 cycles 

if (!countupB[5:4]) begin if ((midibyte_1C==midi_fifo[countupB[3:0]]) && (midi_fifoB[countupB[3:0]]) )     fifo_write<=1;  
else countupB<=countupB+1; end // test if note already on

if (countupB[4] ) begin  if (!midi_fifoB[countupB[3:0]])  fifo_write<=1; else countupB<=countupB+1;  end // test for empty slot
if (countupB[5] )   fifo_write<=1;   // simply write to first position and reset counter 

end // input test and write, max 

end // CLK

reg fifo_write; 
reg fifo_read;  


// Note section  ,, VELOCITY or note is getting stuck, works ok at slow but not when both are pressed at the same time 

reg [21:0]readRam; // midibyte_2[35:29]	,midibyte_1[28:22]	, note_reset[21:4],notecounter[3:0
reg [21:0] readRom [0:15]; //store read_byte out values
reg [6:0] midibyte_1ram[0:15]; 
reg [6:0] midibyte_2ram[0:15]; 



always@( posedge Clk ) begin  
//modulate_con<=modulate; // send wire to reg
//modulate_conB<=modulateB; 
lfoShape <=12'hFFF-  adsrCounter ;		 // read envelope from generator
lfoShapeB <=12'hFFF-  adsrCounterB ; 	// read envelope from generator




if (!read_byteA && fifo_read ) begin // only for one cycle between sends , fifo note out section, exit ofter 1 steps , need to only trigger when fifo_write or play_note

midibyte_2[6:0] <=midi_fifoB[countup[3:0]]; // spit out velocity values 
midibyte_1[6:0] <=midi_fifo[countup[3:0]];
note_oncounter<=countup[3:0]; 

read_byteA<=1;
end


if (read_byteA==1)  // convert midi value for counter, triggered when note value arrives
 begin 

note_resetA <= freqCounter[midibyte_1]; // the value is 50m/(notefreq*16) 4khz highest note	
read_byteA<=2; 
tempCounter<=0;
end

if (read_byteA==2 ) begin // apply when note counter is at zero

read_byteA<=0;  // clear buffers , takes about 10* number of notes cycles to get here 

midibyte_2ram[note_oncounter]<=midibyte_2; //1 |OFF|
midibyte_1ram[note_oncounter]<=midibyte_1; // store midibyte 1 to ram |OFF|
readRam[21:3]<=note_resetA; //3
readRam[3:0]<=note_oncounter; //4 write all incoming values to reg

if (note_oncounter==0 ) begin // write osc values, keep these for saw for now
note_resetB[17:0] <=note_resetA;
note_velocityB[6:0] <=midibyte_2 ; 

end // counterB[0]

if (note_oncounter==1 ) begin  
  note_resetC[17:0] <= note_resetA; 
note_velocityC[6:0] <=midibyte_2;
//modulate_conTempB<=(arraywave_c [arrayPointA]);
end

end // read_byte 

if (read_byteA==0) readRom[readRam[3:0]]<=readRam ; // write read_byte process values to readrom , end of line 




// play_note 
/*
if (!modulate_con) triggerA<=1; 
if (modulate_con== modulate_conTemp    ) triggerA<=0;  // saw output pwm (volt control)

if (!modulate_conB) triggerB<=1; 
if (modulate_conB==modulate_conTempB ) triggerB<=0;  // saw output pwm (volt control)  , clicking for now(less)

if (!counterB) begin pwmLFO<=1; pwmLFOB<=1; end  // go high at zero pwm value , envelope , ,constant frequency
if (counterB[11:00]==(lfoShape[11:0]))  pwmLFO<=0;  // go low at note  , envelope, lfoShape pwm is vca 

if (counterB[11:00]==(lfoShapeB[11:0]))  pwmLFOB<=0;  // go low at note  , envelope, lfoShape pwm is vca 

if ((&counterB) && ((&lfoShape)!=1)) begin  pwmLFO<=0;  end    // go low at end 
if ((&counterB) && ((&lfoShapeB)!=1)) begin  pwmLFOB<=0;  end    // go low at end 



if (modulate==0) note_velocity1<= |note_velocityB;
if (modulateB==0) note_velocity2<= |note_velocityC;	 	// send velocity when zero cross  
*/



end // CLK





// analogue encoder SPI input 50m/512 or 90khz or 45ksps , all good 


reg [9:0] analog_in; // analogue line in capture

always @ (posedge Clk) begin  // needs to be   a minimum speed pr it fails
//CLK1<=~CLK1; // clk output flip
//if ((pot_counter[5:0]==42)&& (!pot_counter[15:9])) pot_counterB<=pot_counter[8:6]; else if (pot_counter[5:0]==42)  pot_counterB<=0;  // every 1000 loops do one full 8 steps
if (counterB[3:0]==4'b1111) begin // at /16 1.5mhz with 50khz sample rate
pot_counter<=pot_counter+1;  // clk for spi adc

end 


//if (internalSeq==1) begin  internalSeq<=2;  end  
case (pot_counter[5:0]) // this is long, no adder

//6'd0 : begin end // resourcehog
6'd0 : begin CS1<=0;Din<=1; end // start
6'd2 : begin Din<=1; end  // single input
6'd4 : Din<=pot_counterB[2];   
6'd6 : Din<=pot_counterB[1];   // note_sequence
6'd8 : Din<=pot_counterB[0];
6'd15 : pot_temp[9]<=Dout;  
6'd17 : pot_temp[8]<=Dout;
6'd19 : pot_temp[7]<=Dout;
6'd21 : pot_temp[6]<=Dout;
6'd23 : pot_temp[5]<=Dout;
6'd25 : pot_temp[4]<=Dout;
6'd27 : pot_temp[3]<=Dout;
6'd29 : pot_temp[2]<=Dout;
6'd31 : pot_temp[1]<=Dout;
6'd33 : begin pot_temp[0]<=Dout; note_sequenceB<=note_sequence; end  // store temp note_sequence
   // will write extra values if not stopped, can change while writing !
6'd34 :  begin   CS1<=1;  end //begin pot_temp<=pot_store[pot_counterB]+pot_temp ; end  // reset CS ,filter pot_temp 
6'd38 : begin   if (pot_counterB[2]) pot_tempB<=~pot_temp;   else pot_tempB<=pot_temp;          end // wrong  store data , count one up on inputs, flip second row, 10 bit
6'd39 : if (!pot_counterB) analog_in<=pot_tempB; // capture line in 10bit
6'd40 :	pot_store[pot_counterB] <=pot_tempB[9:2] ;	//overflowing ,,, if ((pot_tempB!=pot_store[pot_counterB]+1) | 	(pot_tempB!=pot_store[pot_counterB]-1)) 
6'd42 : if (!pot_counter[15:9]) pot_counterB<=pot_counter[8:6]; else pot_counterB<=0; 

endcase

 


sequencer; 
end  //clk

reg [1:0] noteFlag;
// internal sequencer 


task sequencer; // read_byteA gets stuck
begin 
// if (internalSeq && play_note) internalSeq<=0; 

if (noteFlag==2'b11) noteFlag[0]<=0;  // reset to 0 if both are switched on , after writing midibyte, too slow 


if (tempo_counterB[12]) begin tempo_counter<=tempo_counter-1;  tempo_counterB<=2499;  end else tempo_counterB<=tempo_counterB-1;  // this is constant


if (tempo_counter[14])  begin tempo_counter<=tempo_calc; note_sequence<=note_sequence+1; // count up noteseuence
internal_velocity[6] <=~note_sequence[0]; // if note_sequence is even velocity is 64  
tempo_calc<=arraytempo[tempo];
  noteFlag[0]<=1;  if (!note_sequence[0])internal_note<=(pot_store[note_sequenceB[3:1] ] )>>1 ;  end  

  // running but not counting, switch on bit 0 

//if (BUTTONREADA) LED[4:0]<=BUTTONREADA[4:0];  
LED[7:1] <=buttonHolder[7:1];  // changed 

//LED<=(1<<note_sequence[3:1]) * internal_velocity[6] ; 
end
endtask 


// bcd converter 


always @(posedge Clk) begin // works great , can be made faster
//if (!tempo_counter)    // grab tempo ms value , value is 0.1 ms, important skips without it


//if (!counterB[8] && counterB[0]) internal_note<=(pot_store[note_sequenceB[3:1]+8 ] +internal_note) >>1 ; very resource heavy




if (!bcd_counter)   begin  bcd_reader<=internal_note;  end // uses resources
//if (!bcd_counter)    bcd_reader<=pot_store[counterB[6:4] ]; // uses resources


//tempo<= 60+(pot_store[7]>>1);   //calculate tempo value from pot0  , better here less slices, tempo is 8 bit
// disable for now tempo<=60+(buttonHolder[4:0]<<2);  // get tempo from buttons

tempo<=20+(buttonHolder[4:0]<<2);  // get tempo from buttons


case (counterB[3:0])  // working ok now  
4'd0 : begin if (bcd_counter==11) begin  bcd_counter<=0; bcd_storeB<=bcd_store; bcd_store<=0; note_sequenceC<=note_sequence;  end end // reset counter and write values , can be read only when counterB[3:0] true 

1 : bcd_temp<=bcd_store[3:0]; 
3 : bcd_store[3:0]<=bcd_temp;

4 : bcd_temp<=bcd_store[7:4];
6 : bcd_store[7:4]<=bcd_temp;
7 : bcd_temp<=bcd_store[11:8];
9 : bcd_store[11:8] <=bcd_temp;
10 : bcd_temp<=bcd_store[15:12];
12 : bcd_store[15:12]<=bcd_temp; 
13 : bcd_store<=bcd_store<<1;  
14 : bcd_store[0]<=bcd_reader[10-bcd_counter];  
15 :  bcd_counter<=bcd_counter+1;	
default : if (bcd_temp[3] | (bcd_temp[2] && bcd_temp[1:0])) bcd_temp<=bcd_temp+3;
endcase 

end






task notecounter1; // counter for note1 , put these before calls  , needs to be fast
begin 
//if ((note_resetB>>1)==Note) pwmHalf<=~pwmHalf; //flip half interpolate for sine
if (Note[17])  // this is simple freq out only for analogue, correct note hz 
begin
 Note<=note_resetB[17:5] ;
 //modulate<=modulate+1'b1;  // set more than for skipping values , less than 1 for longer wavelenght needs to detect zero csoss
// pwmHalf<=0; 
end else Note<=Note-1;
end 
endtask



task notecounter2; 
begin
if (NoteB[17]) 
begin
 NoteB<=note_resetC[17:5];
// modulateB<=modulateB+1'b1;  // set more than for skipping values , less than 1 for longer wavelenght needs to detect zero csoss 2
end else NoteB<=NoteB-1;
end
endtask

// Audio section also old adsr values 

// velocity 
reg [15:0] lfoShape;
reg [15:0] lfoShapeB;
reg [6:0] note_velocityB; // note velocity storage 6-0 7 is empty flag 
reg [6:0] note_velocityC; // note velocity storage 6-0 7 is empty flag 
reg note_velocity1; // toggle note on off
reg note_velocity2;
reg [31:0] ADSR1;  //adsr total value 8 x4 bit for envelope generator
reg [31:0] ADSR2; 
reg adsrTrigger; // start envelope 
reg adsrTriggerB;
reg [11:0] adsrCounter; // adsr up counter for time
reg [11:0] adsrCounterB;
reg [3:0] adsrStep; // step through attack - release
reg [3:0] adsrStepB;
reg [15:0] sustain; // sustain level
reg [15:0] sustainB; 






//wire [3:0] modulateCounterB=modulateCounter; 
reg [21:0] modulate[0:15] ; // sine modulate counter input ram , 9 bit for value , 10bit for flip
reg [3:0] modulateCounter;  
reg [9:0] modulateB;    // modulate 2
reg [6:0] modulate_con;  // reg for modulate 
reg [5:0] modulate_conB; // second reg modulate
reg [5:0] modulate_conTemp;
reg [5:0] modulate_conTempB;

 reg [10:0] delay_clock; 
wire signed [16:0] delay_input;
reg signed [16:0] delay_output;
reg signed [16:0] delay_outputB;
reg signed [16:0] delay_feedback; 
reg signed [16:0] delay_hold; 
reg delay_enable;
reg  [9:0] delay_pointerB; ;  //out pointer gives a an adding effect with positive values like feedback
reg [9:0] delay_pointerA; 
wire [9:0] delay_pointer;
wire signed  [15:0] pwm_valuetemp;
reg signed [16:0] delay_ramB [0:1023];
wire  [9:0] addrb  = delay_pointerA [9:0] ; // output
reg delay_writeB;
wire  [15:0] delay_out= delay_output +16'sh7FFF ; //= delay_output+16'sh7FFF;  // convert back to unsigned value , works ok at 1.5v



// wire [15:0] pwm_value;     //=  (pwm_valueB[20]) ? 	(16'shFFFF -(~pwm_valueB)+1) : (16'shFFFF+pwm_valueB) ;	 //needs proper formula ; // audio out input            
reg  signed [16:0] pwm_valueB; //signed
 
//wire [7:0] pwm_valueP  =  	 (pwmHalf) ?  ((divide+divideP)>>1) : divide      ; //interpolated value by 2
reg   [15:0] arrayoutC; 
 wire signed   [16:0] divide_temp =   (modulate_temp[9]) ?   (~(arrayoutC)+1):arrayoutC   ;  //  flips 2's compliment on negative
//wire [15:0] noteZero[counterB[6:3]] = modulate_temp[9]; // zero cross flag 

wire signed [16:0] divide_tempB;  //signed
assign divide_tempB=divide_temp; 
reg [15:0] divideBit; // zero cross  reg from divide 
reg signed [16:0] divide [0:15] ; // divide out ram
wire [9:0] modulate_temp=modulate_fine[21 : 12] ;  // not synced to clock ? 

reg noteZeroB; 
reg [6:0]  velocity_temp; // hold pipe velocity 
reg signed [20:0] pwm_accu; // collect average , no issues with scaling
reg [16:0] pwmAout;
reg [21:0] modulate_fine; // high count adder 

//reg signed [16:0] adsr_ram [0:15] ;  // store modified divide out value

reg [11:0]  adsr_counterhold [0:15] ; // ram adsr counter hold, start function , msb for enable count 0 off 1 on , trigger from  midibyte_2ram 
wire [3:0] select_rom  =counterB[6:3] ; // counter for 0-15 for notes ,everywhere , should be fine as its outside 2:0 
reg [11:0] adsr_hold; //temp store for adsr_counterhold
 
reg [6:0] adsr_velocityhold; //grab velocity for adsr
reg [3:0] adsr_clock; // free running adsr clock
//reg [7:0] adsr_clockhold[0:15]; // holds initial clock count per not , clear when finished
reg adsr_clockB; 
wire signed [16:0] analog_inS= (analog_in<<6) - 16'sh7FFF; 
reg adsr_stop; 
wire adsr_noteon=|adsr_velocityhold[6:0];  // key on 
reg signed [16:0] adsr_divide; //divide holder
//reg [11:0] new_adsrB = 12'b110110111110;  // 4*3 bits (8 count) 
//reg [11:0] new_adsr [0:15]; // compare adsrvalues to keep track when similar stop ,except for sustain 

//reg signed [7:0] adsr_level [0:15]; // calculated level from time and adsr 
//reg [6:0] adsr_levelB;
//reg [2:0] adsr_position [0:15]  ; // set where to be in calucaltions , keep track

assign pwm_value [15:0]  = pwm_valueB[15:0] + 16'sh7FFF; 


always @ (posedge Clk    ) begin 








if (counterB[8] && !adsr_clockB)   begin adsr_clockB<=1; adsr_clock<=adsr_clock+1; adsr_stop<=0; end // divide counterB by 10 bit  1m length

if (counterB[7]) adsr_stop<=1; //reset at the end 1
if (!counterB[8] && adsr_clockB) adsr_clockB<=0; // long 2



if ((!adsr_clock) && adsr_clockB && (!adsr_stop) )begin // this is long but stop after finish , sloow



case (counterB[2:0]) // adsr , working good

1  :	begin  adsr_hold <=adsr_counterhold[select_rom];	adsr_velocityhold<=midibyte_2ram[select_rom]; end
2 : if ( adsr_noteon && !adsr_hold) adsr_hold<=1; // start adsr if on and zero value
3 : if (adsr_hold[11:4]==254) adsr_hold<=0; // release finish if note off
4 : begin if (( adsr_noteon && adsr_hold[11:4]!=176) | (!adsr_noteon && adsr_hold))   adsr_counterhold[select_rom] <=  adsr_hold + 8; 
else adsr_counterhold[select_rom] <=  adsr_hold;  end   // hold sustain or go to release ,add adsr rate here if running , do it 16 times  , 12 bit |OFF|

endcase

end 

case (counterB[6:0])  // saw output settings inc pwm for volt control , set for note 0 for now


3 : begin arrayPointA[4:0]<= 31-(modulate_con[6:2]); end // get note from ram


4 : begin modulate_conTemp[4:0]<=(arraywave_c [arrayPointA]);  end // store volt value , not source of clicking
5 : begin if (!modulate_temp[9:5])  triggerA<=1  ;  if (modulate_temp[9:5]==modulate_conTemp[4:0]) triggerA<=0; end // triggerA output





endcase





case (counterB [2:0]) // in=modulate[9:0], out = divide [15:0], 16 pos , write wave values > divide ram, modulate > wave values

0  : modulate_con<=midibyte_1ram[select_rom] ;
1	:	modulate_fine <=  modulate[select_rom] + (modulate_array[modulate_con]);  //modulate adder ,grab old value 16 bit values
2   :begin  modulate[select_rom] <=modulate_fine;   end // store value again 
3  : begin arrayoutC <=arraywave_b[modulate_temp[8:0]] ;   end // case use comma, shifted to mid, always has value, might need to slight shift phase
5  : begin  divide[select_rom] <=divide_temp; end  // write divide to store for adsr etc |OFF|
 endcase


case (counterB[2:0])  // modify adsr and adder out 
		

0 :  if (!select_rom)begin  pwm_accu<=0; pwm_valueB<=(pwm_accu>>4); end //every 50m/128 ,390khz , level should be ok ,reset at 00
1 :  adsr_divide<=divide[select_rom]; 
6 : adsr_hold <=adsr_counterhold[select_rom]; // not always running here
7 :  pwm_accu<=pwm_accu+((adsr_divide*adsr_list[adsr_hold[11:4]])/128) ; // works ok ,but adds up a lot, 3-4 bits, only adds note ons, this is good!!

endcase

end 
// this should be ok for now, for 16 values , 195khz about(50/256) srate, or 180khz modulate max , need to redo modulate values



always @(posedge Clk ) begin // audio generator
counter <=counter+1; // always use reg for these 






one_bit[17:0]  <= one_bit[16:0] + (pwm_remain[15:0]<<1); //+ (analog_in<<6)  ; // one bit out, only unsigned not oversampling

one_bitB[17:0]  <= one_bitB[16:0] + (delay_out[15:0]<<1) ; // delay_out , unsigned



// retrieve data , back to centre, 15 bit signed 
 // shift below zero

// test if (counterB==10 'b1111111111 ) begin 
// delay_outputB<=delay_output; // store last sample
// delay_ramB[delay_pointerB]<=pwm_valueB;  delay_pointerA<=delay_pointerA+1; delay_pointerB<=delay_pointerB+1;    end // chase loaded memory ring
// else  delay_hold <=delay_ramB [delay_pointerA]  ; // works great leave it alone, single cycle write ,constant read!
 
 //if (counterB[10])  delay_output<=(delay_hold+delay_outputB)/2; else delay_output<=delay_hold; // average of last outpt

delay_output<=delay_hold + (delay_hold-delay_output); // seems smoother this way

end  // end clk loop

always @ (posedge counterB[11])  begin 
delay_outputB<=delay_output; // store last sample
 delay_ramB[delay_pointerB]<=pwm_valueB;  delay_pointerA<=delay_pointerA+1; delay_pointerB<=delay_pointerB+1;     // chase loaded memory ring
 //else  delay_hold <=delay_ramB [delay_pointerA]  ; // works great leave it alone, single cycle write ,constant read!end


end




//assign speaker[7:0] = pwm_valueB  ;  // sine output 
assign PWM=one_bit[17]; // send overflow bit out
//assign PWM=pwmstart; // PWm output , dont need it anymore
assign PWMb=one_bitB[17]; 





 endmodule


