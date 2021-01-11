`timescale 1ns/10ps
module datapath(clk,reset,Instruction,Reg2Loc,ALUSrc,MemtoReg,RegWrite,write_enable,ALUop,
                read_enable,negativeAlu,zeroCurr,zeroAlu,overflowAlu,carryOutAlu,imm12Cntrl,xfer_size,byteLoader,flagSignal,movKCntrl);

        input logic [31:0] Instruction;
        input logic [3:0] xfer_size;
        input logic [2:0] ALUop;
        input logic clk,reset,Reg2Loc,ALUSrc,MemtoReg,RegWrite,write_enable,read_enable,byteLoader,flagSignal,imm12Cntrl,movKCntrl;
        output logic negativeAlu,zeroCurr,zeroAlu,overflowAlu,carryOutAlu;

        logic [63:0] ReadReg1,ReadReg2,Imm12,ALUout,dataMemOut,mux1Out,brAdder9,mux3Out,bytetoRegister;
        logic [4:0] Rd,Rm,Rn;
        logic [5:0] sDist;
        logic overflow,negative,zero,carry_out;
        logic [15:0] Imm16_movK, Imm16_movZ;
        logic [63:0] shamt0, shamt1, shamt2, shamt3, shamtOut, MoveOut;
        logic [63:0] shamtZ0, shamtZ1, shamtZ2, shamtZ3, shamtZOut, MoveZOut;
        logic [1:0] shiftSelect;

        //Assigning the registers to the instructions
        assign Rd = Instruction[4:0];
        assign Rn = Instruction[9:5];
        assign Rm = Instruction[20:16];
        assign shiftSelect = Instruction[22:21];
        assign Imm16_movZ = Instruction[20:5];
        assign Imm16_movK = Instruction[20:5];

        //wires
        wire  [4:0] mux0Out;
        wire [3:0] frOut,frIn;

        //Using the DFF_new module to set flags every clock cycle.
        DFF_new DffZero (.q(zeroAlu), .d(zero), .en(flagSignal), .clk);
	    DFF_new DffOne (.q(negativeAlu), .d(negative), .en(flagSignal), .clk);
	    DFF_new DffTwo (.q(carryOutAlu), .d(carry_out), .en(flagSignal), .clk);
	    DFF_new DffThree (.q(overflowAlu), .d(overflow), .en(flagSignal), .clk);

        //Reg2Loc mux.
        genvar i;
        generate 
            for (i = 0; i < 5; i++) begin: muxRouting
                mux_2_1 mux0 (.in({Rm[i],Rd[i]}),.sel(Reg2Loc),.out(mux0Out[i]));
            end
        endgenerate

        //Reg file
        regfile registers (.ReadRegister1(Rn),.ReadRegister2(mux0Out),.WriteRegister(Rd),
                           .WriteData(mux1Out),.RegWrite(RegWrite),.ReadData1(ReadReg1),.ReadData2(ReadReg2),.clk);

        //Zero extend for input Immediate12.
        zeroExtend addZeroes (.in(Instruction[21:10]),.extend(Imm12));//Zero extender for imm12

        //Sign extend for input Immediate9.
        SignExtend9 extending (.in(Instruction[20:12]), .out(brAdder9));//sign extender for imm9

        
        //MOVZ logic 
        assign shamtZ0 = {{48{1'b0}}, Imm16_movZ}; // LSL 0 
        assign shamtZ1 = {{32{1'b0}},Imm16_movZ, {16{1'b0}}};  // LSL 16
        assign shamtZ2 = {{16{1'b0}},Imm16_movZ,{32{1'b0}}};  // LSL 32
        assign shamtZ3 = {Imm16_movZ,{48{1'b0}}}; // LSL 48
        genvar x;
        generate 
            for (x = 0; x < 64; x++) begin: movZbitSlice
                mux4_1 movZ_Mux (.in({shamtZ3[x], shamtZ2[x], shamtZ1[x], shamtZ0[x]}),.sel(shiftSelect),.out(shamtZOut[x]));
            end
        endgenerate


        //MOVK logic 
        assign shamt0 = {ReadReg2[63:16], Imm16_movK}; // LSL 0 
        assign shamt1 = {ReadReg2[63:32],Imm16_movK,ReadReg2[15:0]};  // LSL 16
        assign shamt2 = {ReadReg2[63:48],Imm16_movK,ReadReg2[31:0]};  // LSL 32
        assign shamt3 = {Imm16_movK,ReadReg2[47:0]}; // LSL 48
        genvar g;
        generate 
            for (g = 0; g < 64; g++) begin: movSlecLines
                mux4_1 movK_Mux (.in({shamt3[g], shamt2[g], shamt1[g], shamt0[g]}),.sel(shiftSelect),.out(shamtOut[g]));
            end
        endgenerate

        //2-to-1 mux for selecting the mov instructions(either mux3output or movk)
        genvar v;
        generate 
            for (v = 0; v < 64; v++) begin: muxRout
                mux_2_1 selectMOVE (.in({shamtOut[v],mux3Out[v]}),.sel(movKCntrl),.out(MoveOut[v]));
            end
        endgenerate


        //Using 4-to-1 for alu source.
        genvar j;
        generate 
            for (j = 0; j < 64; j++) begin: selLines
                                  //MOVZ       //Imm12   //Bradder    //Db(reg ouput)
                mux4_1 mux1 (.in({shamtZOut[j],Imm12[j],brAdder9[j],ReadReg2[j]}),.sel({imm12Cntrl,ALUSrc}),.out(mux3Out[j]));
            end
        endgenerate

        //Alu 
        assign zeroCurr = zero;
        alu operations (.A(ReadReg1),.B(MoveOut),.cntrl(ALUop),.result(ALUout),.negative,.zero,.overflow,.carry_out);

        //MemtoReg mux
        genvar k;
        generate 
            for (k = 0; k < 64; k++) begin: memToRegRout
                mux_2_1 mux1 (.in({bytetoRegister[k],ALUout[k]}),.sel(MemtoReg),.out(mux1Out[k]));
            end
        endgenerate

        //Datamemory
        datamem datamemory (.address(ALUout),.write_enable,.read_enable,.write_data(ReadReg2),.clk,.xfer_size,.read_data(dataMemOut));

        //Selecting 1 byte(LDURB) vs 8 bytes(LDUR)
        wire [7:0] dataStorage;
		wire [63:0] dataInsert;
        assign dataStorage = dataMemOut[7:0];
			
		  assign dataInsert = {{56{1'b0}}, {dataStorage}}; 
        genvar n;
        generate 
            for (n = 0; n < 64; n++) begin: byteLines          
                mux_2_1 byteMux (.in({dataMemOut[n], dataInsert[n]}), .sel(byteLoader), .out(bytetoRegister[n]));
            end
        endgenerate
endmodule


module datapath_testbench();
        logic [31:0] Instruction;
        logic [3:0] xfer_size;
        logic [2:0] ALUop;
        logic clk,reset,Reg2Loc,ALUSrc,MemtoReg,RegWrite,write_enable,read_enable,byteLoader,flagSignal,imm12Cntrl;
        logic negativeAlu,zeroAlu,overflowAlu,carryOutAlu;

        datapath dut (.clk,.reset,.Instruction,.Reg2Loc,.ALUSrc,.MemtoReg,.RegWrite,.write_enable,.ALUop,
                .read_enable,.negativeAlu,.zeroAlu,.overflowAlu,.carryOutAlu,.imm12Cntrl,.xfer_size,.byteLoader,.flagSignal);

        parameter ClockDelay = 10000;
        initial begin // Set up the clock
	    clk <= 0;
	    forever #(ClockDelay/2) clk <= ~clk;
	    end 

        initial begin
               // SUBS X2, X0, X1      // X2 =  2
        Instruction = 32'b11101011000000010000000000000010;xfer_size = 4'bxxxx;
        ALUop = 3'b011;Reg2Loc = 1'b1;ALUSrc = 1'b0;MemtoReg = 1'b0;RegWrite = 1'b1;write_enable = 1'b0;
        read_enable = 1'b0; byteLoader = 1'bx; flagSignal = 1'b1; imm12Cntrl =1'b0; @(posedge clk);  

               // SUBS X3, X1, X2      // X3 = -3
        Instruction = 32'b11101011000000100000000000100011;xfer_size = 4'bxxxx;
        ALUop = 3'b011;Reg2Loc = 1'b1;ALUSrc = 1'b0;MemtoReg = 1'b0;RegWrite = 1'b1;write_enable = 1'b0;
        read_enable = 1'b0; byteLoader = 1'bx; flagSignal = 1'b1; imm12Cntrl =1'b0; @(posedge clk);  

                // SUBS X4, X3, X1      // X4 = -2
        Instruction = 32'b11101011000000010000000001100100;xfer_size = 4'bxxxx;
        ALUop = 3'b011;Reg2Loc = 1'b1;ALUSrc = 1'b0;MemtoReg = 1'b0;RegWrite = 1'b1;write_enable = 1'b0;
        read_enable = 1'b0; byteLoader = 1'bx; flagSignal = 1'b1; imm12Cntrl =1'b0; @(posedge clk);  


        // ADDI X1, X0, #1   // X1 = 1
        Instruction = 32'b10010001000000000000010000000001 ;xfer_size = 4'bxxxx;
        ALUop = 3'b010;Reg2Loc = 1'b1;ALUSrc = 1'b0;MemtoReg = 1'b0;RegWrite = 1'b1;write_enable = 1'b0;
        read_enable = 1'b0; byteLoader = 1'bx; flagSignal = 1'b0; imm12Cntrl =1'b1; @(posedge clk);   

        //ADDI  X2, X1, #1   // X2 = 2
        Instruction = 32'b10010001000000000000010000100010 ;xfer_size = 4'bxxxx;
        ALUop = 3'b010;Reg2Loc = 1'b1;ALUSrc = 1'b0;MemtoReg = 1'b0;RegWrite = 1'b1;write_enable = 1'b0;
        read_enable = 1'b0; byteLoader = 1'bx; flagSignal = 1'b0; imm12Cntrl =1'b1; @(posedge clk);   

        // ADDI X3, X1, #2      // X3 = 3
        Instruction = 32'b10010001000000000000100000100011  ;xfer_size = 4'bxxxx;
        ALUop = 3'b010;Reg2Loc = 1'b1;ALUSrc = 1'b0;MemtoReg = 1'b0;RegWrite = 1'b1;write_enable = 1'b0;
        read_enable = 1'b0; byteLoader = 1'bx; flagSignal = 1'b0; imm12Cntrl =1'b1; @(posedge clk); 

        // ADDI X4, X0, #4      // X4 = 4
        Instruction = 32'b10010001000000000000100000100011  ;xfer_size = 4'bxxxx;
        ALUop = 3'b010;Reg2Loc = 1'b1;ALUSrc = 1'b0;MemtoReg = 1'b0;RegWrite = 1'b1;write_enable = 1'b0;
        read_enable = 1'b0; byteLoader = 1'bx; flagSignal = 1'b0; imm12Cntrl =1'b1; @(posedge clk); 
        $stop;
        
        end
endmodule





