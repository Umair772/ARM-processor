
`timescale 1ns/10ps
module regfile(ReadRegister1,ReadRegister2,WriteRegister,WriteData,RegWrite,ReadData1,ReadData2,clk);
	input logic [4:0] ReadRegister1,ReadRegister2,WriteRegister;
	input logic [63:0] WriteData;
	input logic RegWrite;
	output logic [63:0] ReadData1,ReadData2;
	input logic clk;
	logic [31:0]decoderOutput;
	logic [31:0][63:0]ffout;
	logic [31:0][63:0]finalReg;
	logic [63:0][31:0]switch;
	
	decoder_5_to_32 decode(.enable(RegWrite),.in(WriteRegister),.out(decoderOutput));

	regs registerFiles(.writeData(WriteData),.enable(decoderOutput),.clk,.out(ffout));

	genvar m,l;
	generate
		for(m = 0; m < 32; m++) begin:switchReg
			for(l = 0; l < 64; l++) begin: switchBit
				assign switch[l][m] = finalReg[m][l];
			end
		end
	endgenerate


  
  	genvar n;
	generate
		for(n = 0; n < 64; n++) begin: eachBit
			mux32_1 mux1(.in(switch[n][31:0]),.sel(ReadRegister1[4:0]),.out(ReadData1[n]));
			mux32_1 mux2(.in(switch[n][31:0]),.sel(ReadRegister2[4:0]),.out(ReadData2[n]));
		end

	endgenerate


	genvar i;
	generate
		for(i = 0; i < 31; i++) begin: eachReg
			assign finalReg[i][63:0] = ffout[i][63:0];
		end
	endgenerate
			
	
	assign finalReg[31][63:0] = 64'b0; //hardwring the 31's bit to 0.
	
endmodule


// Test bench for Register file
`timescale 1ns/10ps

module regfile_Test_bench(); 		

	parameter ClockDelay = 5000;

	logic	[4:0] 	ReadRegister1, ReadRegister2, WriteRegister;
	logic [63:0]	WriteData;
	logic 			RegWrite, clk;
	logic [63:0]	ReadData1, ReadData2;

	integer i;

	// Your register file MUST be named "regfile".
	// Also you must make sure that the port declarations
	// match up with the module instance in this stimulus file.
	regfile dut (.ReadData1, .ReadData2, .WriteData, 
					 .ReadRegister1, .ReadRegister2, .WriteRegister,
					 .RegWrite, .clk);

	// Force %t's to print in a nice format.
	initial $timeformat(-9, 2, " ns", 10);

	initial begin // Set up the clock
		clk <= 0;
		forever #(ClockDelay/2) clk <= ~clk;
	end

	initial begin
		// Try to write the value 0xA0 into register 31.
		// Register 31 should always be at the value of 0.
		RegWrite <= 5'd0;
		ReadRegister1 <= 5'd0;
		ReadRegister2 <= 5'd0;
		WriteRegister <= 5'd31;
		WriteData <= 64'h00000000000000A0;
		@(posedge clk);
		
		$display("%t Attempting overwrite of register 31, which should always be 0", $time);
		RegWrite <= 1;
		@(posedge clk);

		// Write a value into each  register.
		$display("%t Writing pattern to all registers.", $time);
		for (i=0; i<31; i=i+1) begin
			RegWrite <= 0;
			ReadRegister1 <= i-1;
			ReadRegister2 <= i;
			WriteRegister <= i;
			WriteData <= i*64'h0000010204080001;
			@(posedge clk);
			
			RegWrite <= 1;
			@(posedge clk);
		end

		// Go back and verify that the registers
		// retained the data.
		$display("%t Checking pattern.", $time);
		for (i=0; i<32; i=i+1) begin
			RegWrite <= 0;
			ReadRegister1 <= i-1;
			ReadRegister2 <= i;
			WriteRegister <= i;
			WriteData <= i*64'h0000000000000100+i;
			@(posedge clk);
		end
		$stop;
	end
endmodule
