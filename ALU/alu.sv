`timescale 1ns/10ps
module alu(A,B,cntrl,result,negative,zero,overflow,carry_out);
    input logic [63:0]A,B;
    input logic [2:0]cntrl;
    output logic [63:0] result;
    output logic negative,zero,overflow,carry_out;
    logic [63:0]aluCin; 

    Bit_alu aluSmall0 (.a(A[0]),.b(B[0]),.out(result[0]),.Cin(cntrl[0]),.Cout(aluCin[0]),.en(cntrl[2:0]));
    genvar i;
    generate 
        for (i = 1; i <= 62; i++) begin: consAlu
            Bit_alu larAlu (.a(A[i]), .b(B[i]),.out(result[i]),.Cin(aluCin[i-1]),.Cout(aluCin[i]),.en(cntrl[2:0]));
        end
    endgenerate
    Bit_alu aluSmall1 (.a(A[63]),.b(B[63]),.out(result[63]),.Cin(aluCin[62]),.Cout(carry_out),.en(cntrl[2:0]));

    assign negative = result[63];
    xor #50 overflowFlag(overflow,carry_out,aluCin[62]);
    NOR_64to1 mod0 (.in(result[63:0]),.out(zero));

endmodule 


// Test bench for ALU
`timescale 1ns/10ps

// Meaning of signals in and out of the ALU:

// Flags:
// negative: whether the result output is negative if interpreted as 2's comp.
// zero: whether the result output was a 64-bit zero.
// overflow: on an add or subtract, whether the computation overflowed if the inputs are interpreted as 2's comp.
// carry_out: on an add or subtract, whether the computation produced a carry-out.

// cntrl			Operation						Notes:
// 000:			result = B						value of overflow and carry_out unimportant
// 010:			result = A + B
// 011:			result = A - B
// 100:			result = bitwise A & B		value of overflow and carry_out unimportant
// 101:			result = bitwise A | B		value of overflow and carry_out unimportant
// 110:			result = bitwise A XOR B	value of overflow and carry_out unimportant

module alu_testbench();

	parameter delay = 100000;

	logic		[63:0]	A, B;
	logic		[2:0]		cntrl;
	logic		[63:0]	result;
	logic					negative, zero, overflow, carry_out ;

	parameter ALU_PASS_B=3'b000, ALU_ADD=3'b010, ALU_SUBTRACT=3'b011, ALU_AND=3'b100, ALU_OR=3'b101, ALU_XOR=3'b110;
	

	alu dut (.A, .B, .cntrl, .result, .negative, .zero, .overflow, .carry_out);

	// Force %t's to print in a nice format.
	initial $timeformat(-9, 2, " ns", 10);

	integer i;
	logic [63:0] test_val;
	initial begin
	
		$display("%t testing PASS_A operations", $time);
		cntrl = ALU_PASS_B;
		for (i=0; i<100; i++) begin
			A = $random(); B = $random();
			#(delay);
			assert(result == B && negative == B[63] && zero == (B == '0));
		end
		
		$display("%t testing addition", $time);
		cntrl = ALU_ADD;
		A = 64'h0000000000000001; B = 64'h0000000000000001;
		#(delay);
		assert(result == 64'h0000000000000002 && carry_out == 0 && overflow == 0 && negative == 0 && zero == 0);
		A = 64'b1000000000000000000000000000000000000000000000000000000000000000; B = 64'b1000000000000000000000000000000000000000000000000000000000000000;
		#(delay);
		assert(result == 64'd87 && carry_out == 1 && overflow == 1 && negative == 0 && zero == 0);

        $display("%t testing subtraction", $time);
		cntrl = ALU_SUBTRACT;
		A = 64'h0000000000000001; B = 64'h0000000000000001;
		#(delay);
		assert(result == 64'h0000000000000000 && carry_out == 0 && overflow == 0 && negative == 0 && zero == 1);

        $display("%t testing bitwise and", $time);
		cntrl = ALU_AND;
		A = 64'd5; B = 64'd4;
		#(delay);
		assert(result == 64'd4 && carry_out == 0 && overflow == 0 && negative == 0 && zero == 0);

        $display("%t testing zero", $time);
		cntrl = ALU_AND;
		A = 64'd0; B = 64'd0;
		#(delay);
		assert(result == 64'd0 && carry_out == 0 && overflow == 0 && negative == 0 && zero == 1);

        $display("%t testing bitwise OR", $time);
		cntrl = ALU_OR;
		A = 64'd5; B = 64'd4;
		#(delay);
		assert(result == 64'd7 && carry_out == 0 && overflow == 0 && negative == 0 && zero == 0);

        $display("%t testing bitwise XOR", $time);
		cntrl = ALU_XOR;
		A = 64'd5; B = 64'd4;
		#(delay);
		assert(result == 64'd1 && carry_out == 0 && overflow == 0 && negative == 0 && zero == 0);

        $display("%t testing negative flag", $time);
		cntrl = ALU_PASS_B;
		A = 64'd0; B = 64'b1000000000000000000000000000000000000000000000000000000000000000;
		#(delay);
		assert(result == 64'd1 && carry_out == 0 && overflow == 0 && negative == 1 && zero == 0);
	end
endmodule
