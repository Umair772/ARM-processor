module control(Instruction, negativeFlag, overflowFlag, zero,Cout, ALUOp, xfer_size, Reg2Loc, ALUSrc, MemtoReg, RegWrite, write_en, Brtaken, UncondBr, Imm12cntrl, flagSignal, read_enable, byteLoader, zeroCurr, movKCntrl);
    input logic [31:0] Instruction;
    input logic negativeFlag, overflowFlag ,zero,Cout,zeroCurr;
    output logic [2:0] ALUOp;
    output logic [3:0] xfer_size;
    output logic Reg2Loc, ALUSrc, MemtoReg, RegWrite, write_en, Brtaken, UncondBr, Imm12cntrl, flagSignal, read_enable, byteLoader,movKCntrl;
    wire xorBranch;
    logic [0:8] MoveK_Indicate;

  enum logic [10:0]  {
   ADDI=11'b1001000100x,
        ADDS=11'b10101011000,
        B   =11'b000101xxxxx,
        B_LT=11'b01010100xxx,
        CBZ =11'b10110100xxx,
        LDUR=11'b11111000010,
        LDURB=11'b00111000010,
        MOVK=11'b111100101xx,
        MOVZ=11'b110100101xx,
        STUR = 11'b11111000000,
        STURB =11'b00111000000,
        SUBS = 11'b11101011000

  } opCode;

    xor mod1 (xorBranch,negativeFlag,overflowFlag);

    assign MoveK_Indicate = 9'b111100101;
    //Triggers the byteloader signal based on the instruction op code.
    assign byteLoader = ((Instruction[31:21] == LDUR) ? 1'b1 : 1'b0);
    //Triggers the movKCntrl signal based on the instruction op code.
    assign movKCntrl = ((Instruction[31:23] == MoveK_Indicate) ? 1'b1 : 1'b0);

	always_comb begin
		casex (Instruction[31:21])
	//ADDI Rd, Rn, Imm12: Reg[Rd] = Reg[Rn] + ZeroExtend(Imm12)	      
		ADDI:    begin   Reg2Loc = 1'b0; 
    	                 ALUSrc = 1'b0;      
                         MemtoReg = 1'b0;
                         RegWrite = 1'b1;                 
                         write_en = 1'b0; 
                         Brtaken = 1'b0;
                         UncondBr = 1'bx;
                         ALUOp = 3'b010; 
                         Imm12cntrl = 1'b1; 
                         flagSignal = 1'b0; 
                         read_enable = 1'b0;
                         xfer_size = 4'bxxxx; 
                 end

        //ADDS Rd, Rn, Rm: Reg[Rd] = Reg[Rn] + Reg[Rm]. Set flags. 
        ADDS:   begin    Reg2Loc = 1'b1; 
    	                 ALUSrc = 1'b0;      
                         MemtoReg = 1'b0;
                         RegWrite = 1'b1;                 
                         write_en = 1'b0;
                         Brtaken = 1'b0;
                         UncondBr = 1'bx;
                         ALUOp = 3'b010; 
                         Imm12cntrl = 1'b0; 
                         flagSignal = 1'b1;
                         read_enable = 1'b0;
                         xfer_size = 4'bxxxx; 
                 end

        //B Imm26: PC = PC + SignExtend(Imm26 << 2). 
        B:      begin    Reg2Loc = 1'bx; 
    	                 ALUSrc = 1'bx;      
                         MemtoReg = 1'bx;
                         RegWrite = 1'b0;                 
                         write_en = 1'b0;
                         Brtaken = 1'b1;
                         UncondBr = 1'b1;
                         ALUOp = 3'bxxx; 
                         Imm12cntrl = 1'b0; 
                         flagSignal = 1'b0;
                         read_enable = 1'b0;
                         xfer_size = 4'bxxxx; 
                 end
        
        //B.LT Imm19: If (flags.negative != flags.overflow) PC = PC + SignExtend(Imm19<<2). 
        B_LT:     begin  Reg2Loc = 1'bx; 
    	                 ALUSrc = 1'bx;      
                         MemtoReg = 1'bx;
                         RegWrite = 1'b0;                 
                         write_en = 1'b0;
                         Brtaken = xorBranch;
                         UncondBr = (Brtaken ? 1'b0 : 1'bx);
                         ALUOp = 3'bxxx; 
                         Imm12cntrl = 1'b0; 
                         flagSignal = 1'b0;
                         read_enable = 1'b0;
                         xfer_size = 4'bxxxx; 
                 end

        //CBZ Rd, Imm19: If (Reg[Rd] == 0) PC = PC + SignExtend(Imm19<<2). 
	CBZ:      begin  Reg2Loc = 1'b0; 
    	                 ALUSrc = 1'b0;      
                         MemtoReg = 1'bx;
                         RegWrite = 1'b0;                 
                         write_en = 1'b0;
                         Brtaken = zeroCurr;
                         UncondBr = (Brtaken ? 1'b0 : 1'bx);
                         ALUOp = 3'b000;  
                         Imm12cntrl = 1'b0; 
                         flagSignal = 1'b0;
                         read_enable = 1'b0;
                         xfer_size = 4'bxxxx; 
                 end
        //LDUR Rd, [Rn, #Imm9]: Reg[Rd] = Mem[Reg[Rn] + SignExtend(Imm9)]. 
        LDUR:      begin  Reg2Loc = 1'bx; 
    	                 ALUSrc = 1'b1;      
                         MemtoReg = 1'b1;
                         RegWrite = 1'b1;                 
                         write_en = 1'b0;
                         Brtaken = 1'b0;
                         UncondBr = 1'bx;
                         ALUOp = 3'b010; 
                         Imm12cntrl = 1'b0; 
                         flagSignal = 1'b0;
                         read_enable = 1'b1;
                         xfer_size = 4'b1000; 

                 end
        //LDURB Rd, [Rn, Imm9]: Reg[Rd] = {56’b0, Mem[Reg[Rn] + SignExtend(Imm9)][7:0]}. 
        LDURB:      begin  Reg2Loc = 1'bx; 
    	                 ALUSrc = 1'b1;      
                         MemtoReg = 1'b1;
                         RegWrite = 1'b1;                 
                         write_en = 1'b0;
                         Brtaken = 1'b0;
                         UncondBr = 1'bx;
                         ALUOp = 3'b010;  
                         Imm12cntrl = 1'b0; 
                         flagSignal = 1'b0;
                         read_enable = 1'b1; 
                         xfer_size = 4'b0001; 
                    
                         
                 end
        
         //MOVK Rd, Imm16, LSL Shamt: Reg[Rd][16*Shamt+15:16*Shamt] = Imm16. 
         MOVK:      begin  Reg2Loc = 1'b0; 
    	                 ALUSrc = 1'b1;      
                         MemtoReg = 1'b0;
                         RegWrite = 1'b1;                 
                         write_en = 1'b0;
                         Brtaken = 1'b0;
                         UncondBr = 1'bx;
                         ALUOp = 3'b000; 
                         Imm12cntrl = 1'b1; 
                         flagSignal = 1'b0;
                         read_enable = 1'b0; 
                         xfer_size = 4'bxxxx; 
                 end

        //MOVZ Rd, Imm16, LSL Shamt: Reg[Rd] = Imm16 << (16*Shamt). 
         MOVZ:      begin  Reg2Loc = 1'b0; 
    	                 ALUSrc = 1'b1;      
                         MemtoReg = 1'b0;
                         RegWrite = 1'b1;                 
                         write_en = 1'b0;
                         Brtaken = 1'b0;
                         UncondBr = 1'bx;
                         ALUOp = 3'b000; 
                         Imm12cntrl = 1'b1; 
                         flagSignal = 1'b0;
                         read_enable = 1'b0; 
                         xfer_size = 4'bxxxx; 
                 end

        //STUR Rd, [Rn, #Imm9]: Mem[Reg[Rn] + SignExtend(Imm9)] = Reg[Rd]. 
         STUR:      begin  Reg2Loc = 1'b0; 
    	                 ALUSrc = 1'b1;      
                         MemtoReg = 1'bx;
                         RegWrite = 1'b0;                 
                         write_en = 1'b1;
                         Brtaken = 1'b0;
                         UncondBr = 1'bx;
                         ALUOp = 3'b010; 
                         Imm12cntrl = 1'b0; 
                         flagSignal = 1'b0;
                         read_enable = 1'b0; //if read_enable == 1
                         xfer_size = 4'b1000; 
                         
                         
                 end

       // STURB Rd, [Rn, Imm9]: Mem[Reg[Rn] + SignExtend(Imm9)][7:0] = Reg[Rd][7:0]. 
         STURB:      begin  Reg2Loc = 1'b0; 
    	                 ALUSrc = 1'b1;      
                         MemtoReg = 1'bx;
                         RegWrite = 1'b0;                 
                         write_en = 1'b1;
                         Brtaken = 1'b0;
                         UncondBr = 1'bx;
                         ALUOp = 3'b010; 
                         Imm12cntrl = 1'b0; 
                         flagSignal = 1'b0;
                         read_enable = 1'b0; //if read_enable == 1
                         xfer_size = 4'b0001; 
                         
                 end
                 
        //SUBS Rd, Rn, Rm: Reg[Rd] = Reg[Rn] - Reg[Rm]. Set flags. 
         SUBS:      begin  Reg2Loc = 1'b1; 
    	                   ALUSrc = 1'b0;      
                           MemtoReg = 1'b0;
                           RegWrite = 1'b1;                 
                           write_en = 1'b0;
                           Brtaken = 1'b0;
                           UncondBr = 1'bx;
                           ALUOp = 3'b011;  //Subtract
                           Imm12cntrl = 1'b0; 
                           flagSignal = 1'b1;
                           read_enable = 1'b0;
                           xfer_size = 4'bxxxx;
                 end
        
		default:  begin    Reg2Loc = 1'bx; 
    	                   ALUSrc = 1'bx;      
                           MemtoReg = 1'bx;
                           RegWrite = 1'b0;                 
                           write_en = 1'b0;
                           Brtaken = 1'bx;
                           UncondBr = 1'bx;
                           ALUOp = 3'bxxx;  
                           Imm12cntrl = 1'b0; 
                           flagSignal = 1'b0;
                           read_enable = 1'b0;
                           xfer_size = 4'bxxxx;
                 end   
                  
		endcase 
	end 
 
endmodule 

module control_testbench();
    logic [31:21] Instruction;
    logic negativeFlag, overflowFlag, zero;
    logic [2:0] ALUOp;
    logic [3:0] xfer_size;
    logic Reg2Loc, ALUSrc, MemtoReg, RegWrite, write_en, Brtaken, UncondBr, Imm12cntrl, flagSignal, read_enable, byteLoader, zeroCurr, movKCntrl;

    control dut (.Instruction, .negativeFlag, .overflowFlag, .zero, .ALUOp, .xfer_size, .Reg2Loc, .ALUSrc, .MemtoReg, .RegWrite, .write_en, .Brtaken, .UncondBr, .Imm12cntrl, .flagSignal, .read_enable, .byteLoader, .zeroCurr,.movKCntrl);


    initial begin
        Instruction = 11'b1001000100x; #1000;   // ADDI
        Instruction = 11'b10101011000; #1000;   // ADDS
        Instruction = 11'b000101xxxxx; #1000;   // B
        Instruction = 11'b01010100xxx; negativeFlag = 1'b0; overflowFlag = 1'b0;    #1000;    // B.LT
                                       negativeFlag = 1'b0; overflowFlag = 1'b1;    #1000; 
                                       negativeFlag = 1'b1; overflowFlag = 1'b0;    #1000; 
                                       negativeFlag = 1'b1; overflowFlag = 1'b1;    #1000; 
                                       negativeFlag = 1'b0; overflowFlag = 1'b0;    #1000; 
        Instruction = 11'b10110100xxx; zero = 1'b1;  #1000;    // CBZ
                                       zero = 1'b0;  #1000; 
        Instruction = 11'b11111000010; #1000;    // LDUR
        Instruction = 11'b111000010xx; #1000;    // LDUR
        //MOVK
        //MOVZ
        Instruction = 11'b1111100000;  #1000; // STUR
        Instruction = 11'b111000000xx; #1000; // STURB
        Instruction = 11'b11101011000; #1000; // SUBS
        Instruction = 11'b11101000000; #1000; // default
        Instruction = 11'b11101000111; negativeFlag = 1'b0; overflowFlag = 1'b0;    #1000;  // default
                                       negativeFlag = 1'b0; overflowFlag = 1'b1;    #1000; 
                                       negativeFlag = 1'b1; overflowFlag = 1'b0;    #1000; 
                                       negativeFlag = 1'b1; overflowFlag = 1'b1;    #1000; 
                                       negativeFlag = 1'b0; overflowFlag = 1'b0;    #1000; 
        Instruction = 11'b10001011000; zero = 1'b1;  #1000; // default
                                       zero = 1'b0;  #1000;
        Instruction = 11'b11111111111; #1000;   // default
        Instruction = 11'b00000000000; #1000;   // default       
        $stop;
    end 
endmodule 