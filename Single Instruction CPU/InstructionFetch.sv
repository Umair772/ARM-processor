`timescale 1 ns / 10 ps
module InstructionFetch #(parameter TotalBit=64)(clk,reset,brtaken,uncondBr,Instr);

input logic clk,reset,brtaken,uncondBr;
output logic [31:0] Instr;
logic [TotalBit - 1:0] newPC,oldPC,unCondBr,condBr,normBranch,condSeOut,brSeOut,shiftOutCond,shiftOutBr;
logic negative,zero,overflow,carry_out;


// The program counter
programCounter #(.TotalBit(64)) PC (.in(newPC),.out(oldPC),.clk,.reset);

//Grabbing the instructions from the instruction memory using the program counter.
instructmem outputInstr (.address(oldPC),.instruction(Instr),.clk);

//We input the shifted and signextended values for branch and unconditional branch and output the updated PC.
BranchInstructionsOrNot muxes (.addrCond(shiftOutCond),.addrBr(shiftOutBr),.normPC(normBranch),.justPC(oldPC),.uncondBr(uncondBr),.brtaken(brtaken),.addrInstrOut(newPC));

//Adding 4 to the PC.
alu adder (.A(oldPC),.B(64'd4),.cntrl(3'b010),.result(normBranch),.negative,.zero,.overflow,.carry_out);

//Sign extending the constants for the branch and uncondtional branch
SignExtend SE (.condAdder19(Instr[23:5]),.BrAdder26(Instr[25:0]),.adder19(condSeOut),.adder26(brSeOut));

//Shifting the outputs from the sign extend module
assign shiftOutCond = {condSeOut[61:0], {2{1'b0}}}; // LSL 4
assign shiftOutBr = {brSeOut[61:0], {2{1'b0}}}; // LSL 4

endmodule

module InstructionFetch_testbench();
     logic clk,reset,brtaken,uncondBr;
     logic [31:0] Instr;

    InstructionFetch #(.TotalBit(64)) dut (.clk,.reset,.brtaken,.uncondBr,.Instr);

    parameter ClockDelay = 1000;
    initial begin // Set up the clock
	clk <= 0;
	forever #(ClockDelay/2) clk <= ~clk;
	end
	
     initial begin
     clk = 1'b1; brtaken = 1'b0; uncondBr = 1'b0; reset = 1'b1; @(posedge clk); //1
     clk = 1'b1; brtaken = 1'b0; uncondBr = 1'b0; reset = 1'b0; @(posedge clk); //2
     clk = 1'b1; brtaken = 1'b0; uncondBr = 1'b0; reset = 1'b0; @(posedge clk); //3
     clk = 1'b1; brtaken = 1'b0; uncondBr = 1'b0; reset = 1'b0; @(posedge clk); //4
     clk = 1'b1; brtaken = 1'b0; uncondBr = 1'b0; reset = 1'b0; @(posedge clk); //5
     clk = 1'b1; brtaken = 1'b0; uncondBr = 1'b0; reset = 1'b0; @(posedge clk); //6
     clk = 1'b1; brtaken = 1'b1; uncondBr = 1'b1; reset = 1'b0; @(posedge clk); //7
     clk = 1'b1; brtaken = 1'b1; uncondBr = 1'b1; reset = 1'b0; @(posedge clk); //8
     clk = 1'b1; brtaken = 1'b1; uncondBr = 1'b1; reset = 1'b0; @(posedge clk); //9
     clk = 1'b1; brtaken = 1'b1; uncondBr = 1'b1; reset = 1'b0; @(posedge clk); //10
     clk = 1'b1; brtaken = 1'b1; uncondBr = 1'b1; reset = 1'b0; @(posedge clk); //11
     clk = 1'b1; brtaken = 1'b1; uncondBr = 1'b1; reset = 1'b0; @(posedge clk); //12
	  
     $stop;
     end
endmodule 






