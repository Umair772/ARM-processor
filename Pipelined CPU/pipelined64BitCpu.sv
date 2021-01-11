        `timescale 1 ns / 10 ps
		  module pipelinedCPU_64Bit(clk, reset);

        input logic clk,reset;

        //Control wires
        logic BrTakenIF, UncondBrIF,isBrIF;
		  
		/*IF ------> RF control signals*/
		logic ALUSrcIF;
		logic	MemtoRegIF;
		logic Reg2LocIF;
        logic read_enableIF;
		logic flagSignalIF;
		logic write_enableIF;
		logic byteLoaderIF;
		logic movKCntrlIF;
		logic imm12CntrlIF;
		logic RegWriteIF;
        logic [2:0] ALUopIF;
        logic [3:0] xfer_sizeIF;


        // DataPath/control input/output
        logic [31:0] Instruction,InstrToRegF;
        logic [3:0] xfer_sizeRF;
        logic [2:0] ALUopRF;
        logic Reg2LocRF,ALUSrcRF,MemtoRegRF,RegWriteRF,write_enableRF,read_enableRF,byteLoaderRF,flagSignalRF,imm12CntrlRF,movKCntrlRF,isBr,zeroCheckForABr,RegWriteWB;
        //logic negativeAluRF,zeroCurr,zeroAluRF,overflowAluRF,carryOutAluRF;
        logic zero_datapath,negative_datapath,carry_out_datapath,overflow_datapath;
        logic zero_DFFOUT,negative_DFFOUT,carry_out_DFFOUT,overflow_DFFOUT;
        logic zero_FINAL,negative_FINAL,carry_out_FINAL,overflow_FINAL;

        //DataPath logics(wires)
        logic [63:0] ReadReg1,ReadReg2,Imm12,ALUout,dataMemOut,mux1Out,brAdder9,mux3Out,bytetoRegister,pipeWBReg;
        logic [4:0] Rd,Rm,Rn,writeRegToRegFileWB;
        logic [5:0] sDist;
        //logic overflow,negative,zero,carry_out;
        logic [15:0] Imm16_movK, Imm16_movZ;
        logic [63:0] shamt0, shamt1, shamt2, shamt3, shamtOut, MoveOut;
        logic [63:0] shamtZ0, shamtZ1, shamtZ2, shamtZ3, shamtZOut, MoveZOut;
        logic [1:0] shiftSelect;

        //IF logics
        //logic [31:0] Instr;
        logic [63:0] newPC,oldPC,unCondBr,condBr,normBranch,condSeOut,brSeOut,shiftOutCond,shiftOutBr,outputPC;
        logic negative_current,zero_current,overflow_current,carry_out_current; //Ignore these flags
        logic negative_4,zero_4,overflow_4,carry_out_4;
		  
		  
        logic BrTakenRF;
        logic UncondBrRF;
		logic isBrRF;
		  
		logic BrTakenEX;
		logic UncondBrEX;
		logic isBrEX;
		  
		  
		//logics
		logic [4:0] writeRegToRegFileEX,writeRegToRegFileRF;
		logic ALUSrcEX;
		logic imm12CntrlEX;
		logic RegWriteEX;
		logic RegWriteMEM;
		logic [4:0] writeRegToRegFileMEM;

        //Assigning the registers to the instructions
        assign Rd = InstrToRegF[4:0];
        assign Rn = InstrToRegF[9:5];
        assign Rm = InstrToRegF[20:16];
        assign shiftSelect = InstrToRegF[22:21];
        assign Imm16_movZ = InstrToRegF[20:5];
        assign Imm16_movK = InstrToRegF[20:5];
		  
        //Instruction fetch stuff
        // The program counter
        programCounter #(.TotalBit(64)) PC (.in(newPC),.out(oldPC),.clk,.reset);

        //Grabbing the instructions from the instruction memory using the program counter.
        instructmem outputInstr (.address(oldPC),.instruction(Instruction),.clk);

        //We input the shifted and signextended values for branch and unconditional branch and output the updated PC.

        //BranchInstructionsOrNot muxes (.addrCond(shiftOutCond),.addrBr(shiftOutBr),.normPC(normBranch),.justPC(oldPC),.uncondBr(UncondBr),.brtaken(BrTaken),.addrInstrOut(newPC));

        logic [63:0] unCondOutput;
        logic [63:0] unCondBranchAdder;

        genvar bi;
        generate
        for (bi = 0; bi <= 63; bi++) begin: eachMuxUncond
            mux_2_1 unCondBrTaken (.in({shiftOutBr[bi],shiftOutCond[bi]}),.sel(UncondBrRF),.out(unCondOutput[bi]));
        end
        endgenerate

        // PC = PC + SignExtend((BrAddr26)/(CondAddr19))<<2.
        alu adder_Current (.A(outputPC),.B(unCondOutput),.cntrl(3'b010),.result(unCondBranchAdder),.negative(negative_current),.zero(zero_current),.overflow(overflow_current),.carry_out(carry_out_current));

        //Using the branch taken control signal we are either branching or just adding 4 to the program counter.
        genvar on;
        generate 
        for (on = 0; on <= 63; on++) begin: eachMuxBrNorm
            mux_2_1 brNormTaken (.in({unCondBranchAdder[on],normBranch[on]}),.sel(BrTakenRF),.out(newPC[on]));
        end
        endgenerate 

        //Adding 4 to the PC.
        alu adder_4 (.A(outputPC),.B(64'd4),.cntrl(3'b010),.result(normBranch),.negative(negative_4),.zero(zero_4),.overflow(overflow_4),.carry_out(carry_out_4));

        //Sign extending the constants for the branch and uncondtional branch
        SignExtend SE (.condAdder19(InstrToRegF[23:5]),.BrAdder26(InstrToRegF[25:0]),.adder19(condSeOut),.adder26(brSeOut));

        //Shifting the outputs from the sign extend module
        assign shiftOutCond = {condSeOut[61:0], {2{1'b0}}}; // LSL 2
        assign shiftOutBr = {brSeOut[61:0], {2{1'b0}}}; // LSL 2

        //Passing the oldPC through a DFF
        logic [63:0] pipePC;
        //logic [63:0] outputPC;
        genvar bFF;
        generate
        for (bFF = 0; bFF < 64; bFF++) begin: branchFlipFlop
            D_FF BFlops (.q(pipePC[bFF]),.d(oldPC[bFF]),.clk,.reset);
        end
        endgenerate

        //Determines the program counter based on the isBr signal from the control unit
        genvar bOut;
        generate
        for (bOut = 0; bOut < 64; bOut++) begin: branchOut
            mux_2_1 brRouting (.in({pipePC[bOut],oldPC[bOut]}),.sel(isBrRF),.out(outputPC[bOut]));  
        end
        endgenerate

        /* Pipes: IF ----> RegFile*/
        //wire [31:0] InstrToRegF;
        genvar hk;
        generate 
        for (hk= 0; hk < 32; hk++) begin: IfToRegPipe
            D_FF flipFlops (.q(InstrToRegF[hk]),.d(Instruction[hk]),.clk,.reset);
        end
        endgenerate
		  
        //Control stuff
        //Control module:
        //Inputs: Takes in the instruction from the instruction fetch module, the flags from the datapath.
        //Outputs: Ouputs all signals needed for the datapath for each instruction.
        control signals (.Instruction(InstrToRegF), .negativeFlag(negative_FINAL), .overflowFlag(overflow_FINAL), 
        .zero(zero_FINAL), .Cout(carry_out_FINAL), 
        .ALUOp(ALUopRF), .xfer_size(xfer_sizeRF), 
        .Reg2Loc(Reg2LocRF), .ALUSrc(ALUSrcRF), .MemtoReg(MemtoRegRF), .RegWrite(RegWriteRF), 
        .write_en(write_enableRF), .Brtaken(BrTakenRF), .UncondBr(UncondBrRF), .Imm12cntrl(imm12CntrlRF), .flagSignal(flagSignalRF), 
        .read_enable(read_enableRF), .byteLoader(byteLoaderRF),.zeroCurr(zeroCheckForABr),
        .movKCntrl(movKCntrlRF),.isBr(isBrRF));

        //Datapath stuff 
        //pipeping the flagsignal from RF to EX
        wire flagSignalEX;
        D_FF pipeControlSignal17 (.q(flagSignalEX),.d(flagSignalRF),.clk,.reset);


        //Using the DFF_new module to set flags every clock cycle.
        DFF_new DffZero (.q(zero_DFFOUT), .d(zero_datapath), .en(flagSignalEX), .clk);
        DFF_new DffOne (.q(negative_DFFOUT), .d(negative_datapath), .en(flagSignalEX), .clk);
        DFF_new DffTwo (.q(carry_out_DFFOUT), .d(carry_out_datapath), .en(flagSignalEX), .clk);
        DFF_new DffThree (.q(overflow_DFFOUT), .d(overflow_datapath), .en(flagSignalEX), .clk);
		 

        //Reg2Loc mux.
        logic [4:0] mux0Out;
        genvar i;
        generate 
        for (i = 0; i < 5; i++) begin: muxRouting
            mux_2_1 mux0 (.in({Rm[i],Rd[i]}),.sel(Reg2LocRF),.out(mux0Out[i]));
        end
        endgenerate
		  
		  logic [4:0] pipeOutRnIFReg;
        genvar pipeRn;
        generate 
        for (pipeRn = 0; pipeRn < 5; pipeRn++) begin: muxRoutingRn
            D_FF flipFlopsRn (.q(pipeOutRnIFReg[pipeRn]),.d(Rn[pipeRn]),.clk,.reset);
        end
        endgenerate
		  
		  
        //Forwarding unit
        logic [1:0] fwda,fwdb,fwdc;
        logic flagFwdUnit; //Ouput from the forwarding unit   //Change declaration to Ex     //Change pipe name to mem.
        forwardingUnit module01 (.AaRF(mux0Out), .AbRF(Rn), .AwEx(writeRegToRegFileRF), .AwMem(writeRegToRegFileEX), .RegWriteEX(RegWriteEX), .RegWriteMEM(RegWriteMEM), .flagSignalEX(flagSignalEX), .isBr(isBrRF), .movKCntrl(movKCntrlRF), .AlUSrcEX(ALUSrcRF), .imm12CntrlEX(imm12CntrlRF), .fwdA(fwda), .fwdB(fwdb), .fwdC(fwdc), .flagFwdUnit(flagFwdUnit));

        /*----------------Reg/Decode stage--------------------*/
        //Reg file
        regfile registers (.ReadRegister1(Rn),.ReadRegister2(mux0Out),.WriteRegister(writeRegToRegFileMEM),
            .WriteData(pipeWBReg),.RegWrite(RegWriteWB),.ReadData1(ReadReg1),.ReadData2(ReadReg2),.clk);

        //Zero extend for input Immediate12.
        zeroExtend addZeroes (.in(InstrToRegF[21:10]),.extend(Imm12));//Zero extender for imm12

        //Sign extend for input Immediate9.
        SignExtend9 extending (.in(InstrToRegF[20:12]), .out(brAdder9));//sign extender for imm9

        /*Pipes: RegFile ----> Execute stage*/
        //Insert muxes here for forwardng? 2 4-to-1 muxes to select the values from either the execute stage
        //or the mem stage.
        //wire [4:0] writeRegToRegFileEX;
        wire [63:0] exAluPortA;
        wire [63:0] ALUOutMuxWire;
        wire [63:0] memStageMuxWire;
        wire [63:0] exAluPortB;
        logic [63:0] muxFwdAOut;
        logic [63:0] muxFwdBOut;
        logic [63:0] muxFwdCOut;
        logic [63:0] writePortReg;
        //Forwading A and B muxes
        genvar y,s;
        generate 
        for (y = 0; y < 64; y++) begin: forwardingALogics
                                //0     //memStage   //exAlu    //Da(reg ouput)
            mux4_1 mux5 (.in({1'b0,memStageMuxWire[y],ALUOutMuxWire[y],ReadReg1[y]}),.sel(fwda),.out(muxFwdAOut[y]));
        end
        for (s = 0; s < 64; s++) begin: forwardingBLogics
                                //0     //memStage   //exAlu           //
            mux4_1 mux6 (.in({1'b0,memStageMuxWire[s],ALUOutMuxWire[s],MoveOut[s]}),.sel(fwdb),.out(muxFwdBOut[s]));
        end
        endgenerate

        //Checking if Db == 0 for accelerated branches
        NOR_64to1 mod0 (.in(muxFwdBOut),.out(zeroCheckForABr));

        //pipe RF-EX transition
        genvar p,z,f,iterRD,iterRDRFEX;
        generate
        for (p = 0; p < 64; p++) begin: pipeingFlipFlopsA
            D_FF pipeRFEX0 (.q(exAluPortA[p]),.d(muxFwdAOut[p]),.clk,.reset); //Port 1 of ALU
        end
        for (z = 0; z < 64; z++) begin: pipeingFlipFlopsB
            D_FF pipeRFEX1 (.q(exAluPortB[z]),.d(muxFwdBOut[z]),.clk,.reset); //Port 2 of ALU
        end
        for (f = 0; f < 64; f++) begin: pipeingFlipFlopsC
            D_FF pipeRFEX2 (.q(writePortReg[f]),.d(muxFwdCOut[f]),.clk,.reset); //Port write data of the data mem
        end
		  /* RegRd If -------> RF */
        for (iterRD = 0; iterRD < 5; iterRD++) begin: regPipeFlop
            D_FF pipeRFEX3 (.q(writeRegToRegFileRF[iterRD]),.d(Rd[iterRD]),.clk,.reset); //Port datamem to write back of reg RD
        end
		  
		  for (iterRDRFEX = 0; iterRDRFEX < 5; iterRDRFEX++) begin: regPipeFlop21
            D_FF pipeRFEX31 (.q(writeRegToRegFileEX[iterRDRFEX]),.d(writeRegToRegFileRF[iterRDRFEX]),.clk,.reset); //Port datamem to write back of reg RD
        end
		  
        endgenerate
        
        //Pipeing for RF --------> EX stage control signals
        //FlagSignal piped or not?
        //wire ALUSrcEX;
        wire write_enableEX;
        wire MemtoRegEX;
        wire read_enableEX;
		wire byteLoaderEX;
		wire movKCntrlEX;
        wire [2:0] ALUopEX;
        wire [3:0] xfer_sizeEX;
		  
		D_FF pipeControlSignal44 (.q(BrTakenEX),.d(BrTakenRF),.clk,.reset);
        D_FF pipeControlSignal45 (.q(UncondBrEX),.d(UncondBrRF),.clk,.reset);
		D_FF pipeControlSignal46 (.q(isBrEX),.d(isBrRF),.clk,.reset);

        D_FF pipeControlSignal0 (.q(ALUSrcEX),.d(ALUSrcRF),.clk,.reset);
        D_FF pipeControlSignal14 (.q(imm12CntrlEX),.d(imm12CntrlRF),.clk,.reset);
        D_FF pipeControlSignal3 (.q(write_enableEX),.d(write_enableRF),.clk,.reset);
        D_FF pipeControlSignal40 (.q(MemtoRegEX),.d(MemtoRegRF),.clk,.reset);
        D_FF pipeControlSignal5 (.q(read_enableEX),.d(read_enableRF),.clk,.reset);
        D_FF pipeControlSignal60 (.q(RegWriteEX),.d(RegWriteRF),.clk,.reset);
		D_FF pipeControlSignal13 (.q(byteLoaderEX),.d(byteLoaderRF),.clk,.reset);
		D_FF pipeControlSignal26 (.q(movKCntrlEX),.d(movKCntrlRF),.clk,.reset);
		  
        genvar pipeALUopRFEX,pipeXfer_SizeRFEX;
        generate
        for (pipeALUopRFEX = 0; pipeALUopRFEX < 3; pipeALUopRFEX++) begin: ALUopFlops_08
            D_FF pipeControlSignal2 (.q(ALUopEX[pipeALUopRFEX]),.d(ALUopRF[pipeALUopRFEX]),.clk,.reset);
        end
        for (pipeXfer_SizeRFEX = 0; pipeXfer_SizeRFEX < 4; pipeXfer_SizeRFEX++) begin: ALUopFlops_12
            D_FF pipeControlSignal139 (.q(xfer_sizeEX[pipeXfer_SizeRFEX]),.d(xfer_sizeRF[pipeXfer_SizeRFEX]),.clk,.reset);
        end
        endgenerate 
        /*-----------------------------------------------------*/

        /*----------------Execute stage--------------------*/
        //Using 4-to-1 for alu source.
        genvar j;
        generate 
        for (j = 0; j < 64; j++) begin: selLines
                                 //MOVZ       //Imm12   //Bradder    //Db(reg ouput)
            mux4_1 mux1 (.in({shamtZOut[j],Imm12[j],brAdder9[j],ReadReg2[j]}),.sel({imm12CntrlRF,ALUSrcRF}),.out(mux3Out[j]));
        end
        endgenerate

        //Alu 
        //assign zeroCurr = zero_datapath;

        //wire for forwarding
        assign ALUOutMuxWire = ALUout;
        alu operations (.A(exAluPortA),.B(exAluPortB),.cntrl(ALUopEX),.result(ALUout),.negative(negative_datapath),.zero(zero_datapath),.overflow(overflow_datapath),.carry_out(carry_out_datapath));

        //Setting flags using genvar statement
        wire [3:0] flagIn_1,flagIn_2,ForwardFlags;
        //assigning input 1 for mux
        assign flagIn_1[0] = zero_DFFOUT;
        assign flagIn_1[1] = negative_DFFOUT;
        assign flagIn_1[2] = carry_out_DFFOUT;
        assign flagIn_1[3] = overflow_DFFOUT;
        //assigning input 2 for mux
        assign flagIn_2[0] = zero_datapath;
        assign flagIn_2[1] = negative_datapath;
        assign flagIn_2[2] = carry_out_datapath;
        assign flagIn_2[3] = overflow_datapath;
        genvar flagFwd;
        generate 
        for (flagFwd = 0; flagFwd < 4; flagFwd++) begin: memToRegRout
            mux_2_1 muxFwd (.in({flagIn_2[flagFwd],flagIn_1[flagFwd]}),.sel(flagFwdUnit),.out(ForwardFlags[flagFwd]));
        end
        endgenerate
		  
        assign zero_FINAL = ForwardFlags[0];
        assign negative_FINAL = ForwardFlags[1];
        assign carry_out_FINAL = ForwardFlags[2];
        assign overflow_FINAL = ForwardFlags[3];

        //Control signals for Execute stage -----------------> MEM stage.
        wire write_enableMEM;
        wire MemtoRegMEM;
        wire read_enableMEM;
		wire byteLoaderMEM;
		wire movKCntrlMEM;
        wire [3:0] xfer_sizeMEM;

        D_FF pipeControlSignal7 (.q(write_enableMEM),.d(write_enableEX),.clk,.reset);
        D_FF pipeControlSignal8 (.q(MemtoRegMEM),.d(MemtoRegEX),.clk,.reset);
        D_FF pipeControlSignal9 (.q(read_enableMEM),.d(read_enableEX),.clk,.reset);
        D_FF pipeControlSignal10 (.q(RegWriteMEM),.d(RegWriteEX),.clk,.reset);
		D_FF pipeControlSignal11 (.q(byteLoaderMEM),.d(byteLoaderEX),.clk,.reset);
		D_FF pipeControlSignal21 (.q(movKCntrlMEM),.d(movKCntrlEX),.clk,.reset);
		  
        generate
        genvar pipeXfer_SizeMEM;
        for (pipeXfer_SizeMEM = 0; pipeXfer_SizeMEM < 4; pipeXfer_SizeMEM++) begin: ALUopFlops
            D_FF pipeControlSignal11 (.q(xfer_sizeMEM[pipeXfer_SizeMEM]),.d(xfer_sizeEX[pipeXfer_SizeMEM]),.clk,.reset);
        end
        endgenerate 



        /*-----------------------------------------------------*/

        //Forwarding mux for the write data port of datamem
        genvar u;
        generate
        for (u = 0; u < 64; u++) begin: forwardingCLogics
                                //0     //memStage   //exAlu    //Db(reg ouput)
            mux4_1 mux7 (.in({1'b0,memStageMuxWire[u],ALUOutMuxWire[u],ReadReg2[u]}),.sel(fwdc),.out(muxFwdCOut[u]));
        end
        endgenerate

        /* Pipes: Execute stage ----> Mem stage*/
        //wire [4:0] writeRegToRegFileMEM;
        wire [63:0] pipeOutMem;
        wire [63:0] writeDataPipeReg;
        genvar pipeMem,pipeWrite,iterRDEX;
        generate
        for (pipeMem = 0; pipeMem < 64; pipeMem++) begin: pipeingFlipFlopsX
            D_FF pipeEXMEM0 (.q(pipeOutMem[pipeMem]),.d(ALUout[pipeMem]),.clk,.reset);
        end
        for (pipeWrite = 0; pipeWrite < 64; pipeWrite++) begin: pipeingFlipFlopsD
            D_FF pipeEXMEM1 (.q(writeDataPipeReg[pipeWrite]),.d(writePortReg[pipeWrite]),.clk,.reset);
        end
        for (iterRDEX = 0; iterRDEX < 5; iterRDEX++) begin: regPipeFlop12
            D_FF pipeEXMEM2 (.q(writeRegToRegFileMEM[iterRDEX]),.d(writeRegToRegFileEX[iterRDEX]),.clk,.reset);
        end
        endgenerate
        /*----------------Mem stage--------------------*/
        //MemtoReg mux
        //Wire for forwarding
        assign memStageMuxWire = mux1Out;
        genvar k;
        generate 
        for (k = 0; k < 64; k++) begin: memToRegRout13
            mux_2_1 mux1 (.in({bytetoRegister[k],pipeOutMem[k]}),.sel(MemtoRegMEM),.out(mux1Out[k]));
        end
        endgenerate

        //Datamemory
        datamem datamemory (.address(pipeOutMem),.write_enable(write_enableMEM),.read_enable(read_enableMEM),.write_data(writeDataPipeReg),.clk,.xfer_size(xfer_sizeMEM),.read_data(dataMemOut));

        //Selecting 1 byte(LDURB) vs 8 bytes(LDUR)
        wire [7:0] dataStorage;
        wire [63:0] dataInsert;
        assign dataStorage = dataMemOut[7:0];
        assign dataInsert = {{56{1'b0}}, {dataStorage}}; 
        genvar n;
        generate 
        for (n = 0; n < 64; n++) begin: byteLines          
            mux_2_1 byteMux (.in({dataMemOut[n], dataInsert[n]}), .sel(byteLoaderMEM), .out(bytetoRegister[n]));
        end
        endgenerate
        /*-----------------------------------------------------*/
        /* Pipe: Mem stage ----> WB stage*/
        //wire [4:0] writeRegToRegFileWB;
        //wire [63:0] pipeWBReg;
        genvar pipIter,iterRDWB;
        generate
        for (pipIter = 0; pipIter < 64; pipIter++) begin: pipeingFlipFlopsE
            D_FF pipeWB0 (.q(pipeWBReg[pipIter]),.d(mux1Out[pipIter]),.clk,.reset);
        end
        for (iterRDWB = 0; iterRDWB < 5; iterRDWB++) begin: regPipeFlop15
            D_FF pipeWB1  (.q(writeRegToRegFileWB[iterRDWB]),.d(writeRegToRegFileMEM[iterRDWB]),.clk,.reset);
        end
        endgenerate

        //wire RegWriteWB;

        D_FF pipeControlSignal18 (.q(RegWriteWB),.d(RegWriteMEM),.clk,.reset);
        /*------------------------MOVK and MOVZ logic------------------------------*/
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
        assign shamt0 = {muxFwdCOut[63:16], Imm16_movK}; // LSL 0 
        assign shamt1 = {muxFwdCOut[63:32],Imm16_movK,muxFwdCOut[15:0]};  // LSL 16
        assign shamt2 = {muxFwdCOut[63:48],Imm16_movK,muxFwdCOut[31:0]};  // LSL 32
        assign shamt3 = {Imm16_movK,muxFwdCOut[47:0]}; // LSL 48
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
            mux_2_1 selectMOVE (.in({shamtOut[v],mux3Out[v]}),.sel(movKCntrlRF),.out(MoveOut[v]));
        end
        endgenerate

endmodule

module pipelinedCPU_64Bit_testbench();
        logic clk,reset;


        pipelinedCPU_64Bit dut (.clk,.reset);

        parameter ClockDelay = 50000;
        initial begin // Set up the clock
            clk <= 0;
            forever #(ClockDelay/2) clk <= ~clk;
        end

        initial begin
            reset = 1; @(posedge clk);
            reset = 0; @(posedge clk);
            repeat (2000) @(posedge clk);
        $stop;
        end
endmodule 
