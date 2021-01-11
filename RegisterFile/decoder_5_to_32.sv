`timescale 1ps/1fs
module decoder_5_to_32(enable, in, out);
	output logic [31:0] out;
	input logic [4:0] in;
	input logic enable;
	
	logic [1:0] select;

	decoder_1_to_2 decode_select(.enable, .in(in[4]), .out(select[1:0]));
	decoder_4_to_16 decode_1(.enable(select[0]), .in(in[3:0]), .out(out[15:0]));
	decoder_4_to_16 decode_2(.enable(select[1]), .in(in[3:0]), .out(out[31:16]));

endmodule

module decoder_5_to_32_testbench();

 logic [31:0] out;
 logic [4:0] in;
 logic enable;

 decoder_5_to_32 dut (.enable, .in, .out);
 

 initial begin
 enable = 1'b0;
 in = 5'b00000; 	  #10;
 
 in = 5'b00011; 	  #10;
 
 enable = 1'b1;
 in = 5'b00001; 	  #10;  // 20
 
 in = 5'b00010; 	  #10;  // 10

 in = 5'b00011;       #10;
 
 enable = 1'b0;
 
 in = 5'b00001; 	  #10;
 
 enable = 1'b1;
 
 in = 5'b01111; 	  #10;
 
 end



endmodule
