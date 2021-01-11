`timescale 1ns/10ps
module cpu64Bit (clk, reset);
    input logic clk,reset;
    logic [31:0] instruction;
    logic [2:0] ALUop;
    logic [3:0] xfer_size;
    logic Reg2Loc,ALUSrc,MemtoReg,RegWrite,MemWrite,MemRead,BrTaken,uncondBr,negativeAlu,zeroAlu,overflowAlu,carryOutAlu,imm12Cntrl,byteLoader,flagSignal,movKCntrl;
    logic zeroCurr;

    //The instuction fetch module:
    //Inputs: Brtaken(branch taken signal from the control),uncondBr(unconditional branch signal from the control, a clk and rest signal.
    //Ouput: The instruction from the instuction memory. 
    InstructionFetch #(.TotalBit(64)) Instructions (.clk,.reset,.brtaken(BrTaken),.uncondBr(uncondBr),.Instr(instruction));

    //Control module:
    //Inputs: Takes in the instruction from the instruction fetch module, the flags from the datapath.
    //Outputs: Ouputs all signals needed for the datapath for each instruction.
    control signals (.Instruction(instruction), .negativeFlag(negativeAlu), .overflowFlag(overflowAlu), .zero(zeroAlu), .Cout(carryOutAlu), 
							.ALUOp(ALUop), .xfer_size(xfer_size), .Reg2Loc(Reg2Loc), .ALUSrc(ALUSrc), .MemtoReg(MemtoReg), .RegWrite(RegWrite), .write_en(MemWrite), .Brtaken(BrTaken), .UncondBr(uncondBr), .Imm12cntrl(imm12Cntrl), .flagSignal(flagSignal), 
							.read_enable(MemRead), .byteLoader(byteLoader),.zeroCurr(zeroCurr),.movKCntrl(movKCntrl));

    //Datapath module
    //Inputs: Takes in the instruction from the instruction fetch module and all the control signals from the control module.
    //Outputs: All the flags from the ALU as inputs for the control module.
    datapath data (.clk,.reset,.Instruction(instruction),.Reg2Loc(Reg2Loc),.ALUSrc(ALUSrc),.MemtoReg(MemtoReg),
                    .RegWrite(RegWrite),.write_enable(MemWrite),.ALUop(ALUop),
                   .read_enable(MemRead),.negativeAlu(negativeAlu),.zeroCurr(zeroCurr),.zeroAlu(zeroAlu),.overflowAlu(overflowAlu),
                   .carryOutAlu(carryOutAlu),.imm12Cntrl(imm12Cntrl),.xfer_size(xfer_size),.byteLoader(byteLoader),.flagSignal(flagSignal),.movKCntrl(movKCntrl));

endmodule

    
module cpu64Bit_testbench();
    logic clk,reset;


        cpu64Bit dut (.clk,.reset);

        parameter ClockDelay = 10000;
        initial begin // Set up the clock
	    clk <= 0;
	    forever #(ClockDelay/2) clk <= ~clk;
	    end

	initial begin
		reset = 1; @(posedge clk);
		reset = 0; @(posedge clk);
        repeat (1000) @(posedge clk);
      $stop;
    end
endmodule 
