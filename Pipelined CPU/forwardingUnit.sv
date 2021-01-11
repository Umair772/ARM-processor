module forwardingUnit (AaRF, AbRF, AwEx, AwMem, RegWriteEX, RegWriteMEM, flagSignalEX, isBr, movKCntrl, AlUSrcEX, imm12CntrlEX, fwdA, fwdB, fwdC, flagFwdUnit);
    input logic [4:0] AaRF,AbRF, AwEx,AwMem;
    input logic RegWriteEX,RegWriteMEM,flagSignalEX,isBr,movKCntrl,AlUSrcEX, imm12CntrlEX;
    output logic [1:0] fwdA,fwdB,fwdC;
    output logic flagFwdUnit;

    // case when we are forwarding the ALU result and the dataMem Result, otherwise, we do not forward
                    //RegWrite in Ex          //MemRd = Rn        // Rd != X31 
    assign fwdA = ((RegWriteEX == 1'b1) && (AwEx == AbRF) && (AwEx != 5'd31)) ? (2'b01) : 
                    //RegWrite in Mem          //MemRd = Rn        // Rd != X31     //Not in EX forward                                                Da(regOutput)
                  ((RegWriteMEM == 1'b1) && (AwMem == AbRF) && (AwMem != 5'd31)) ? (2'b10) : (2'b00);
    

    // case when we are forwarding the ALU result and the dataMem Result, otherwise, we do not forward
                    //RegWrite in Ex          //MemRd = Rm        // Rd != X31 
    assign fwdB = ((RegWriteEX == 1'b1) && (AwEx == AaRF) && (AwEx != 5'd31) && (movKCntrl == 1'b0) && (imm12CntrlEX == 1'b0) && (AlUSrcEX == 1'b0)) ? (2'b01) : 
                    //RegWrite in Mem          //MemRd = Rm        // Rd != X31     //Not in EX forward                                                Da(regOutput)
                  ((RegWriteMEM == 1'b1) && (AwMem == AaRF) && (AwMem != 5'd31) && (movKCntrl == 1'b0) && (imm12CntrlEX == 1'b0) && (AlUSrcEX == 1'b0)) ? (2'b10) : (2'b00);
    
    // case when we are forwarding the ALU result and the dataMem Result, otherwise, we do not forward
                    //RegWrite in Ex          //MemRd = Rm        // Rd != X31 
    assign fwdC = ((RegWriteEX == 1'b1) && (AwEx == AaRF) && (AwEx != 5'd31)) ? (2'b01) : 
                    //RegWrite in Mem          //MemRd = Rm        // Rd != X31     //Not in EX forward                                                Da(regOutput)
                  ((RegWriteMEM == 1'b1) && (AwMem == AaRF) && (AwMem != 5'd31)) ? (2'b10) : (2'b00);

    assign flagFwdUnit = ((isBr == 1'b1) && (flagSignalEX)) ? (1'b1) : (1'b0);
endmodule

module forwardingUnit_testbench();
    logic [4:0] AaRF,AbRF, AwEx,AwMem;
    logic RegWriteEX,RegWriteMEM,flagSignalEX,isBr,movKCntrl,AlUSrcEX, imm12CntrlEX;
    logic [1:0] fwdA,fwdB,fwdC;
    logic flagFwdUnit;
	 
	 forwardingUnit dut(.AaRF, .AbRF, .AwEx, .AwMem, .RegWriteEX, .RegWriteMEM, .flagSignalEX, .isBr, .movKCntrl, .AlUSrcEX, .imm12CntrlEX, .fwdA, .fwdB, .fwdC, .flagFwdUnit);
	 
	 initial begin
		RegWriteEX = 1'b0; RegWriteMEM =1'b0; AaRF = 5'd1; AbRF = 5'd0; AwEx = 5'd0; AwMem = 5'd10; #1000; // fwd A = 2'b00, fwd B = 2'b00, fwd C = 2'b00
		
		/* forwarding test for A  */
		
		RegWriteEX = 1'b1; RegWriteMEM =1'b0; AaRF = 5'd5; AbRF = 5'd0; AwEx = 5'd5; AwMem = 5'd0; #1000; // fwd A = 2'b01, fwd B = 2'b00, fwd C = 2'b00
		
		RegWriteEX = 1'b0; RegWriteMEM =1'b1; AaRF = 5'd5; AbRF = 5'd0; AwEx = 5'd0; AwMem = 5'd5; #1000; // fwd A = 2'b10, fwd B = 2'b00, fwd C = 2'b00

		RegWriteEX = 1'b1; RegWriteMEM =1'b0; AaRF = 5'd31; AbRF = 5'd0; AwEx = 5'd31; AwMem = 5'd0; #1000; // fwd A = 2'b00, fwd B = 2'b00, fwd C = 2'b00
		
		RegWriteEX = 1'b0; RegWriteMEM =1'b1; AaRF = 5'd31; AbRF = 5'd0; AwEx = 5'd0; AwMem = 5'd31; #1000; // fwd A = 2'b00, fwd B = 2'b00, fwd C = 2'b00
	
		/* forwarding test for B */
		
		RegWriteEX = 1'b0; RegWriteMEM =1'b1; AaRF = 5'd0; AbRF = 5'd10; AwEx = 5'd0; AwMem = 5'd10; movKCntrl = 1'b0; imm12CntrlEX = 1'b0; AlUSrcEX = 1'b0; #1000; // fwd A = 2'b00, fwd B = 2'b10, fwd C = 2'b10
				
		RegWriteEX = 1'b1; RegWriteMEM =1'b0; AaRF = 5'd0; AbRF = 5'd10; AwEx = 5'd10; AwMem = 5'd0; movKCntrl = 1'b0; imm12CntrlEX = 1'b0; AlUSrcEX = 1'b0; #1000; // fwd A = 2'b00, fwd B = 2'b01, fwd C = 2'b01				
		
		RegWriteEX = 1'b1; RegWriteMEM =1'b0; AaRF = 5'd0; AbRF = 5'd31; AwEx = 5'd31; AwMem = 5'd0; movKCntrl = 1'b0; imm12CntrlEX = 1'b0; AlUSrcEX = 1'b0; #1000; // fwd A = 2'b00, fwd B = 2'b00, fwd C = 2'b00				
	 	 
		RegWriteEX = 1'b0; RegWriteMEM =1'b1; AaRF = 5'd0; AbRF = 5'd31; AwEx = 5'd0; AwMem = 5'd31; movKCntrl = 1'b0; imm12CntrlEX = 1'b0; AlUSrcEX = 1'b0; #1000; // fwd A = 2'b00, fwd B = 2'b00, fwd C = 2'b00

		/* forwarding test for C  */
		
		RegWriteEX = 1'b1; RegWriteMEM =1'b0; AaRF = 5'd0; AbRF = 5'd5; AwEx = 5'd5; AwMem = 5'd0; #1000; // fwd A = 2'b00, fwd B = 2'b01, fwd C = 2'b01
		
		RegWriteEX = 1'b0; RegWriteMEM =1'b1; AaRF = 5'd0; AbRF = 5'd5; AwEx = 5'd0; AwMem = 5'd5; #1000; // fwd A = 2'b00, fwd B = 2'b10, fwd C = 2'b10

		RegWriteEX = 1'b1; RegWriteMEM =1'b0; AaRF = 5'd0; AbRF = 5'd31; AwEx = 5'd31; AwMem = 5'd0; #1000; // fwd A = 2'b00, fwd B = 2'b00, fwd C = 2'b00
		
		RegWriteEX = 1'b0; RegWriteMEM =1'b1; AaRF = 5'd0; AbRF = 5'd31; AwEx = 5'd0; AwMem = 5'd31; #1000; // fwd A = 2'b00, fwd B = 2'b00, fwd C = 2'b00

		/* forwarding test for Flag signal */
		
		isBr = 1'b0; flagSignalEX = 1'b0; #1000; // flagFwdUnit = 0;
		isBr = 1'b0; flagSignalEX = 1'b1; #1000; // flagFwdUnit = 0;
		isBr = 1'b1; flagSignalEX = 1'b0; #1000; // flagFwdUnit = 0;
		isBr = 1'b1; flagSignalEX = 1'b1; #1000; // flagFwdUnit = 1;		
	 
		$stop;
	 
	 end
	 
endmodule


