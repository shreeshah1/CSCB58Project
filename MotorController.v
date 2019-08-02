`timescale 1ns / 1ns // `timescale time_unit/time_precision


module MotorController(GPIO, KEY, CLOCK_25, CLOCK_50, LEDR, SW, HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7, LEDG);
	//Initiaizing inputs and outputs
	input [3:0] KEY;
	inout [35:0] GPIO;	
	input [17:0] SW;
	input CLOCK_25;
	output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7;
	input CLOCK_50;
	output [17:0] LEDR, LEDG;
	
	
	//wire that determines when the car needs to stop moving (emergency stop)
	wire stop_signal;
	assign stop_signal = SW[15];
	
	//for future changes/additions (new functionality), output of the obstacle FSM
	wire [3:0] obstacle_out;
	
	assign LEDG[4:0] = GPIO[14:10];
	
	//signal for distance sensing
	wire distance_out;
	
	//calling the sensor_test module in order to detect a distance less than 10cm
   sensor_test s1(.clock25(CLOCK_25), .clock50(CLOCK_50), .gpio(GPIO), .hex0(HEX0), .hex1(HEX1), .hex2(HEX2), .hex3(HEX3), .hex4(HEX4), .hex5(HEX5), .hex6(HEX6), .hex7(HEX7), .distance_out(distance_out));
	
	//for future changes/additions (new functionality)
	wire onleft, onright, revleft, revright;
	reg [3:0] final_out;
	assign onleft = final_out[3];
	assign revleft = final_out[2];
	assign onright = final_out[1];
	assign revright = final_out[0];
	
	//calling module that controls the car movement, essentially the FSM
	control c1(.clk(CLOCK_50), .stop_signal(stop_signal), .distance_signal(distance_out), .out(obstacle_out));
	
	
	//for future changes/additions (new functionality)
	always@(*)
	begin
		if(SW[16] == 1'b1)
			final_out <= obstacle_out;
		//else
			//final_out <= letter_out;	
	end
	
	//assigning outputs to GPIO pins 
	assign GPIO[10] = onleft;
	assign GPIO[11] = revleft;
	assign GPIO[12] = ~revleft;
	assign GPIO[13] = revleft;
	assign GPIO[14] = ~revleft;
			
	assign GPIO[26] = onright;
	assign GPIO[27] = revright;
	assign GPIO[28] = ~revright;
	assign GPIO[29] = revright;
	assign GPIO[30] = ~revright;
	
	
	reg [3:0] letter_out; 
	
	/*
	for future changes/additions (new functionality)
	*/
	
	//speller brian(,spell);
			
			
	/*reg [25:0] letter_chain;
	always@(*)
	begin 
		if (~KEY[1])
			start_writing <= 1'b1;
		if (~KEY[0])
			start_writing <= 1'b0;
			letter_chain[4:0] <= letter_inputs;
			letter_chain <= letter_chain << 5; 
	end*/
	
	//always@(*)
	
endmodule

//for future changes/additions (new functionality)
/*
module speller(
		input start_writing,
		input reg [25:0] letter_chain,
		output ready_next,
		output reg [3:0] letter_out
	);
	
	reg [5:0] letter;
	always @(*)
	begin
		
	end


endmodule
*/
	
	
	
/*Main module containing the FSM. This controls the movement of the bot throughout its run.
	Features :
		1) Emergency stop - activated by giving SW[15] a value of 1
		2) Distance sensor - activated by giving SW[14] a value of 1
		3) Normal functionality - bot moves until distance is detected, afterwhich it stops and turns until distance is
										  no longer detected
*/

module control(
input clk,
    input stop_signal, 
    input distance_signal,
    output reg [3:0] out
    );		

	reg [25:0] counter; //counter used for time based states
	reg [5:0] current_state, next_state; 
	
	
	//initializing and decrementing counter
	always@(posedge clk)
	begin
		if (counter > 1'b0)
			counter <= counter - 1'b1;
		if (current_state == S_BRAKE)
			counter <= BRAKE_TIME;	
		else if (current_state == S_REST)
			counter <= REST_TIME;
		else if (current_state == S_TURN)
			counter <= TURN_TIME;
	end
	
	
	wire t; //time signal sent when counter is 0
	assign t = (counter == 10'b0000000000) ? 1 : 0;
    
   //time values for braking, resting and turning
	localparam	BRAKE_TIME = 26'b10111110101111000010000000,
					REST_TIME = 26'b10111110101111000010000000,
					TURN_TIME = 26'b10111110101111000010000000;
    
	 //states
	 localparam  S_DRIVE         = 5'd0,
                S_BRAKE 		  = 5'd1,
                S_BRAKE_WAIT    = 5'd2,
                S_REST		     = 5'd3,
                S_REST_WAIT     = 5'd4,
                S_TURN  		  = 5'd5,
					 S_TURN_WAIT	  = 5'd6;

	
	//combing all input signals into array
	wire [2:0] std;
	assign std = {stop_signal, t, distance_signal};
              
     
    // Next state logic aka our state table
    always@(*)
    begin: state_table 
            case (current_state)
                S_DRIVE: next_state = (distance_signal || stop_signal) ? S_BRAKE : S_DRIVE; // Loop in current state until distance signal high or emergency stop symbol high
                S_BRAKE: next_state = S_BRAKE_WAIT;
                S_BRAKE_WAIT: next_state = t ? S_REST : S_BRAKE_WAIT; // Loop in current state until timer signal is high
                S_REST: next_state = S_REST_WAIT; 
                S_REST_WAIT: next_state = (t && ~stop_signal) ? S_TURN : S_REST_WAIT; // Loop in current state until timer value is high and emergency stop signal is low
                S_TURN: next_state = S_TURN_WAIT;
					 S_TURN_WAIT:
						
						//case table for when robot is in s_turn_wait 
						begin				
						case(std)
							3'b000: next_state = S_TURN_WAIT;
							3'b001: next_state = S_TURN_WAIT;
							3'b010: next_state = S_DRIVE;
							3'b011: next_state = S_TURN_WAIT;
							3'b100: next_state = S_REST_WAIT;
							3'b101: next_state = S_REST_WAIT;
							3'b110: next_state = S_REST_WAIT;
							3'b111: next_state = S_REST_WAIT;
								default: next_state = S_REST_WAIT;
						endcase
						end
						
            default:     next_state = S_REST;
        endcase
    end // state_table
   

    // Output logic aka all of our datapath control signals
    always @(*)
    begin: enable_signals
			
        case (current_state)
           S_DRIVE: begin
                out <= 4'b1010;
                end
           S_BRAKE: begin
                out <= 4'b1111;
				
                end
           S_BRAKE_WAIT: begin
                out <= 4'b1111;
                end
		   S_REST: begin
                out <= 4'b0000;
                end
           S_REST_WAIT: begin
                out <= 4'b0000;
                end
			S_TURN: begin
                	out <= 4'b0001; //test: swtich back to 1011
                end
			S_TURN_WAIT: begin	
				out <= 4'b1011;
				end

        // default:    // don't need default since we already made sure all of our outputs were assigned a value at the start of the always block
        endcase
    end // enable_signals
   
	
    // current_state registers
    always@(posedge clk)
    begin: state_FFs
            current_state <= next_state;
    end // state_FFS
endmodule


/*
Sensor test module that uses the sensor module in order to send a signal to top level module that distance < 10cm
*/

module sensor_test(clock25, clock50, gpio, hex0, hex1, hex2, hex3, hex4, hex5, hex6, hex7, distance_out);
	input clock25, clock50;
	inout [35:0] gpio;
	output [6:0] hex0, hex1, hex2, hex3, hex4, hex5, hex6, hex7;
	output reg distance_out;
	
	wire hundo, dix ,ult;
	wire [20:0] sensor_output;
	wire [3:0] hundreds, tens, ones;


	always @(*)
	begin
		distance_out <= ult;
	end


	assign hundo = (hundreds == 4'b0000) ? 1 : 0;
	assign dix = (tens < 4'b0001) ? 1 : 0;

	assign ult = hundo && dix;



	usensor sensor_hex0(.distance(sensor_output),
							  .trig(gpio[0]),
							  .echo(gpio[1]),
							  .clock(clock50));
							  					  

	BCD bcd(
	  .binary(sensor_output[7:0]),
	  .Hundreds(hundreds),
	  .Tens(tens),
	  .Ones(ones)
	  );

	hex_display display_hundreds(
	  .IN(hundreds),
	  .OUT(hex2)
	  );

	hex_display display_tens(
	  .IN(tens),
	  .OUT(hex1)
	  );

	hex_display display_ones(
	  .IN(ones),
	  .OUT(hex0)
	  );

	  hex_display display_h3(.IN(4'b1010), .OUT(hex5));
	  
	  hex_display display_h4(.IN(4'b0001), .OUT(hex4));
	  
	  hex_display display_h5(.IN(4'b1110), .OUT(hex3));
	  
			reg [25:0] lights; 
		
	always @(posedge sensor_output[7:0]) 
		case(sensor_output[7:0])
			8'd1 :lights<=26'b10000000000000000000000000;
			8'd2 :lights<=26'b10000000000000000000000000;
			8'd3 :lights<=26'b10000000000000000000000000;
			8'd4 :lights<=26'b10000000000000000000000000;
			8'd5 :lights<=26'b10000000000000000000000000;
			8'd6 :lights<=26'b10000000000000000000000000;
			8'd 7 :lights<=26'b10000000000000000000000000;
			8'd 8 :lights<=26'b11000000000000000000000000;
			8'd 9 :lights<=26'b11000000000000000000000000;
			8'd 10 :lights<=26'b11000000000000000000000000;
			8'd 11 :lights<=26'b11000000000000000000000000;
			8'd 12 :lights<=26'b11000000000000000000000000;
			8'd 13 :lights<=26'b11000000000000000000000000;
			8'd 14 :lights<=26'b11000000000000000000000000;
			8'd 15 :lights<=26'b11100000000000000000000000;
			8'd 16 :lights<=26'b11100000000000000000000000;
			8'd 17 :lights<=26'b11100000000000000000000000;
			8'd 18 :lights<=26'b11100000000000000000000000;
			8'd 19 :lights<=26'b11100000000000000000000000;
			8'd 20 :lights<=26'b11100000000000000000000000;
			8'd 21 :lights<=26'b11100000000000000000000000;
			8'd 22 :lights<=26'b11110000000000000000000000;
			8'd 23 :lights<=26'b11110000000000000000000000;
			8'd 24 :lights<=26'b11110000000000000000000000;
			8'd 25 :lights<=26'b11110000000000000000000000;
			8'd 26 :lights<=26'b11110000000000000000000000;
			8'd 27 :lights<=26'b11110000000000000000000000;
			8'd 28 :lights<=26'b11110000000000000000000000;
			8'd 29 :lights<=26'b11111000000000000000000000;
			8'd 30 :lights<=26'b11111000000000000000000000;
			8'd 31 :lights<=26'b11111000000000000000000000;
			8'd 32 :lights<=26'b11111000000000000000000000;
			8'd 33 :lights<=26'b11111000000000000000000000;
			8'd 34 :lights<=26'b11111000000000000000000000;
			8'd 35 :lights<=26'b11111000000000000000000000;
			8'd 36 :lights<=26'b11111100000000000000000000;
			8'd 37 :lights<=26'b11111100000000000000000000;
			8'd 38 :lights<=26'b11111100000000000000000000;
			8'd 39 :lights<=26'b11111100000000000000000000;
			8'd 40 :lights<=26'b11111100000000000000000000;
			8'd 41 :lights<=26'b11111100000000000000000000;
			8'd 42 :lights<=26'b11111100000000000000000000;
			8'd 43 :lights<=26'b11111110000000000000000000;
			8'd 44 :lights<=26'b11111110000000000000000000;
			8'd 45 :lights<=26'b11111110000000000000000000;
			8'd 46 :lights<=26'b11111110000000000000000000;
			8'd 47 :lights<=26'b11111110000000000000000000;
			8'd 48 :lights<=26'b11111110000000000000000000;
			8'd 49 :lights<=26'b11111110000000000000000000;
			8'd 50 :lights<=26'b11111111000000000000000000;
			8'd 51 :lights<=26'b11111111000000000000000000;
			8'd 52 :lights<=26'b11111111000000000000000000;
			8'd 53 :lights<=26'b11111111000000000000000000;
			8'd 54 :lights<=26'b11111111000000000000000000;
			8'd 55 :lights<=26'b11111111000000000000000000;
			8'd 56 :lights<=26'b11111111000000000000000000;
			8'd 57 :lights<=26'b11111111100000000000000000;
			8'd 58 :lights<=26'b11111111100000000000000000;
			8'd 59 :lights<=26'b11111111100000000000000000;
			8'd 60 :lights<=26'b11111111100000000000000000;
			8'd 61 :lights<=26'b11111111100000000000000000;
			8'd 62 :lights<=26'b11111111100000000000000000;
			8'd 63 :lights<=26'b11111111100000000000000000;
			8'd 64 :lights<=26'b11111111110000000000000000;
			8'd 65 :lights<=26'b11111111110000000000000000;
			8'd 66 :lights<=26'b11111111110000000000000000;
			8'd 67 :lights<=26'b11111111110000000000000000;
			8'd 68 :lights<=26'b11111111110000000000000000;
			8'd 69 :lights<=26'b11111111110000000000000000;
			8'd 70 :lights<=26'b11111111110000000000000000;
			8'd 71 :lights<=26'b11111111111000000000000000;
			8'd 72 :lights<=26'b11111111111000000000000000;
			8'd 73 :lights<=26'b11111111111000000000000000;
			8'd 74 :lights<=26'b11111111111000000000000000;
			8'd 75 :lights<=26'b11111111111000000000000000;
			8'd 76 :lights<=26'b11111111111000000000000000;
			8'd 77 :lights<=26'b11111111111000000000000000;
			8'd 78 :lights<=26'b11111111111100000000000000;
			8'd 79 :lights<=26'b11111111111100000000000000;
			8'd 80 :lights<=26'b11111111111100000000000000;
			8'd 81 :lights<=26'b11111111111100000000000000;
			8'd 82 :lights<=26'b11111111111100000000000000;
			8'd 83 :lights<=26'b11111111111100000000000000;
			8'd 84 :lights<=26'b11111111111100000000000000;
			8'd 85 :lights<=26'b11111111111110000000000000;
			8'd 86 :lights<=26'b11111111111110000000000000;
			8'd 87 :lights<=26'b11111111111110000000000000;
			8'd 88 :lights<=26'b11111111111110000000000000;
			8'd 89 :lights<=26'b11111111111110000000000000;
			8'd 90 :lights<=26'b11111111111110000000000000;
			8'd 91 :lights<=26'b11111111111110000000000000;
			8'd 92 :lights<=26'b11111111111111000000000000;
			8'd 93 :lights<=26'b11111111111111000000000000;
			8'd 94 :lights<=26'b11111111111111000000000000;
			8'd 95 :lights<=26'b11111111111111000000000000;
			8'd 96 :lights<=26'b11111111111111000000000000;
			8'd 97 :lights<=26'b11111111111111000000000000;
			8'd 98 :lights<=26'b11111111111111000000000000;
			8'd 99 :lights<=26'b11111111111111100000000000;
			8'd 100 :lights<=26'b11111111111111100000000000;
			8'd 101 :lights<=26'b11111111111111100000000000;
			8'd 102 :lights<=26'b11111111111111100000000000;
			8'd 103 :lights<=26'b11111111111111100000000000;
			8'd 104 :lights<=26'b11111111111111100000000000;
			8'd 105 :lights<=26'b11111111111111100000000000;
			8'd 106 :lights<=26'b11111111111111110000000000;
			8'd 107 :lights<=26'b11111111111111110000000000;
			8'd 108 :lights<=26'b11111111111111110000000000;
			8'd 109 :lights<=26'b11111111111111110000000000;
			8'd 110 :lights<=26'b11111111111111110000000000;
			8'd 111 :lights<=26'b11111111111111110000000000;
			8'd 112 :lights<=26'b11111111111111110000000000;
			8'd 113 :lights<=26'b11111111111111111000000000;
			8'd 114 :lights<=26'b11111111111111111000000000;
			8'd 115 :lights<=26'b11111111111111111000000000;
			8'd 116 :lights<=26'b11111111111111111000000000;
			8'd 117 :lights<=26'b11111111111111111000000000;
			8'd 118 :lights<=26'b11111111111111111000000000;
			8'd 119 :lights<=26'b11111111111111111000000000;
			8'd 120 :lights<=26'b11111111111111111100000000;
			8'd 121 :lights<=26'b11111111111111111100000000;
			8'd 122 :lights<=26'b11111111111111111100000000;
			8'd 123 :lights<=26'b11111111111111111100000000;
			8'd 124 :lights<=26'b11111111111111111100000000;
			8'd 125 :lights<=26'b11111111111111111100000000;
			8'd 126 :lights<=26'b11111111111111111100000000;
			8'd 127 :lights<=26'b11111111111111111110000000;
			8'd 128 :lights<=26'b11111111111111111110000000;
			8'd 129 :lights<=26'b11111111111111111110000000;
			8'd 130 :lights<=26'b11111111111111111110000000;
			8'd 131 :lights<=26'b11111111111111111110000000;
			8'd 132 :lights<=26'b11111111111111111110000000;
			8'd 133 :lights<=26'b11111111111111111110000000;
			8'd 134 :lights<=26'b11111111111111111111000000;
			8'd 135 :lights<=26'b11111111111111111111000000;
			8'd 136 :lights<=26'b11111111111111111111000000;
			8'd 137 :lights<=26'b11111111111111111111000000;
			8'd 138 :lights<=26'b11111111111111111111000000;
			8'd 139 :lights<=26'b11111111111111111111000000;
			8'd 140 :lights<=26'b11111111111111111111000000;
			8'd 141 :lights<=26'b11111111111111111111100000;
			8'd 142 :lights<=26'b11111111111111111111100000;
			8'd 143 :lights<=26'b11111111111111111111100000;
			8'd 144 :lights<=26'b11111111111111111111100000;
			8'd 145 :lights<=26'b11111111111111111111100000;
			8'd 146 :lights<=26'b11111111111111111111100000;
			8'd 147 :lights<=26'b11111111111111111111100000;
			8'd 148 :lights<=26'b11111111111111111111110000;
			8'd 149 :lights<=26'b11111111111111111111110000;
			8'd 150 :lights<=26'b11111111111111111111110000;
			8'd 151 :lights<=26'b11111111111111111111110000;
			8'd 152 :lights<=26'b11111111111111111111110000;
			8'd 153 :lights<=26'b11111111111111111111110000;
			8'd 154 :lights<=26'b11111111111111111111110000;
			8'd 155 :lights<=26'b11111111111111111111111000;
			8'd 156 :lights<=26'b11111111111111111111111000;
			8'd 157 :lights<=26'b11111111111111111111111000;
			8'd 158 :lights<=26'b11111111111111111111111000;
			8'd 159 :lights<=26'b11111111111111111111111000;
			8'd 160 :lights<=26'b11111111111111111111111000;
			8'd 161 :lights<=26'b11111111111111111111111000;
			8'd 162 :lights<=26'b11111111111111111111111100;
			8'd 163 :lights<=26'b11111111111111111111111100;
			8'd 164 :lights<=26'b11111111111111111111111100;
			8'd 165 :lights<=26'b11111111111111111111111100;
			8'd 166 :lights<=26'b11111111111111111111111100;
			8'd 167 :lights<=26'b11111111111111111111111100;
			8'd 168 :lights<=26'b11111111111111111111111100;
			8'd 169 :lights<=26'b11111111111111111111111110;
			8'd 170 :lights<=26'b11111111111111111111111110;
			8'd 171 :lights<=26'b11111111111111111111111110;
			8'd 172 :lights<=26'b11111111111111111111111110;
			8'd 173 :lights<=26'b11111111111111111111111110;
			8'd 174 :lights<=26'b11111111111111111111111110;
			8'd 175 :lights<=26'b11111111111111111111111110;
			8'd 176 :lights<=26'b11111111111111111111111111;
			8'd 177 :lights<=26'b11111111111111111111111111;
			8'd 178 :lights<=26'b11111111111111111111111111;
			8'd 179 :lights<=26'b11111111111111111111111111;
			8'd 180 :lights<=26'b11111111111111111111111111;
			8'd 181 :lights<=26'b11111111111111111111111111;
			8'd 182 :lights<=26'b11111111111111111111111111;
			8'd 183 :lights<=26'b11111111111111111111111111;
			8'd 184 :lights<=26'b11111111111111111111111111;
			8'd 185 :lights<=26'b11111111111111111111111111;
			8'd 186 :lights<=26'b11111111111111111111111111;
			8'd 187 :lights<=26'b11111111111111111111111111;
			8'd 188 :lights<=26'b11111111111111111111111111;
			8'd 189 :lights<=26'b11111111111111111111111111;
			8'd 190 :lights<=26'b11111111111111111111111111;
			8'd 191 :lights<=26'b11111111111111111111111111;
			8'd 192 :lights<=26'b11111111111111111111111111;
			8'd 193 :lights<=26'b11111111111111111111111111;
			8'd 194 :lights<=26'b11111111111111111111111111;
			8'd 195 :lights<=26'b11111111111111111111111111;
			8'd 196 :lights<=26'b11111111111111111111111111;
			8'd 197 :lights<=26'b11111111111111111111111111;
			8'd 198 :lights<=26'b11111111111111111111111111;
			8'd 199 :lights<=26'b11111111111111111111111111;
			default: lights<=26'b00000000000000000000000000;
		endcase
		
endmodule

module usensor(distance, trig, echo, clock);
  input clock, echo;
  output reg [25:0] distance;
  output reg trig;

  reg [25:0] master_timer;
  reg [25:0] trig_timer;
  reg [25:0] echo_timer;
  reg [25:0] echo_shift10;
  reg [25:0] echo_shift12;
  reg [25:0] temp_distance;
  reg echo_sense, echo_high;

  localparam  TRIG_THRESHOLD = 14'b10011100010000,
				  MASTER_THRESHOLD = 26'b10111110101111000010000000;


  always @(posedge clock)
  begin
	 if (master_timer == MASTER_THRESHOLD)
		begin
		  master_timer <= 0;
		  
		  end
	 else if (trig_timer == TRIG_THRESHOLD || echo_sense)
		begin
		  trig <= 0;
		  echo_sense <= 1;
		  if (echo)
								 begin
					echo_high <= 1;
					echo_timer <= echo_timer + 1;
					//////////////////////////////////////////////////////
					// CLOCK_50 -> 50 000 000 clock cycles per second
					// let n = number of cycles
					// speed of sound in air: 340m/s
					// n / 50 000 000 = num of seconds
					// num of seconds * 340m/s = meters
					// meters * 100 = cm ~ distance to object and back
					// So we divide by 2 to get distance to object
					// 1/ 50 000 000 * 340 * 100 / 2 = 0.00034
					// n * 0.00034 = n * 34/100 000 = n / (100 000/34)
					// = 2941
					// To make up for sensor inaccuracy and simple math
					// we round down to 2900
					temp_distance <= (echo_timer / 2900);
					//////////////////////////////////////////////////////
				 end
		  else
			 begin
				distance <= temp_distance + 2'd2;
				echo_timer <= 0;
				trig_timer <= 0;
				echo_sense <= 0;
			 end
		end
	 else
		begin
		trig <= 1;
		trig_timer <= trig_timer + 1;
		master_timer <= master_timer + 1;
	 end
  end
endmodule

module hex_display(IN, OUT);
	input [3:0] IN;
	 output reg [7:0] OUT;

	 always @(*)
	 begin
		case(IN[3:0])
			4'b0000: OUT = 7'b1000000;
			4'b0001: OUT = 7'b1111001;
			4'b0010: OUT = 7'b0100100;
			4'b0011: OUT = 7'b0110000;
			4'b0100: OUT = 7'b0011001;
			4'b0101: OUT = 7'b0010010;
			4'b0110: OUT = 7'b0000010;
			4'b0111: OUT = 7'b1111000;
			4'b1000: OUT = 7'b0000000;
			4'b1001: OUT = 7'b0011000;
			4'b1010: OUT = 7'b0001000;
			4'b1011: OUT = 7'b0000011;
			4'b1100: OUT = 7'b1000110;
			4'b1101: OUT = 7'b0100001;
			4'b1110: OUT = 7'b0000110;
			4'b1111: OUT = 7'b0001110;

			default: OUT = 7'b0111111;
		endcase

	end
endmodule

	// BINARY TO BCD CONVERSION ALGORITHM
	// CODE REFERENCED FROM
	// http://www.eng.utah.edu/~nmcdonal/Tutorials/BCDTutorial/BCDConversion.html
module BCD (
  input [7:0] binary,
  output reg [3:0] Hundreds,
  output reg [3:0] Tens,
  output reg [3:0] Ones
  );

  integer i;
  always @(binary)
  begin
	 //set 100's, 10's, and 1's to 0
	 Hundreds = 4'd0;
	 Tens = 4'd0;
	 Ones = 4'd0;

	 for (i = 7; i >=0; i = i-1)
	 begin
		//add 3 to columns >= 5
		if (Hundreds >= 5)
		  Hundreds = Hundreds + 3;
		if (Tens >= 5)
		  Tens = Tens + 3;
		if (Ones >= 5)
		  Ones = Ones + 3;

		//shift left one
		Hundreds = Hundreds << 1;
		Hundreds[0] = Tens[3];
		Tens = Tens << 1;
		Tens[0] = Ones[3];
		Ones = Ones << 1;
		Ones[0] = binary[i];
	 end
  end
endmodule