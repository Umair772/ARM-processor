`timescale 1ps/1fs
module regs(writeData,enable,clk,out);
    input logic [63:0]writeData; //The 64-bit data in
	input logic clk;
    input logic [31:0]enable; //output from the decoder
    output logic [31:0][63:0]out;

   logic [31:0][63:0] memory;
	
    genvar i,j;
    generate
        for(i=0; i<32; i++) begin:eachReg
            for(j=0; j<64; j++) begin:eachDff
                DFF_new register(.q(memory[i][j]),.d(writeData[j]),.en(enable[i]),.clk);
            end
        end
    endgenerate 
	 
//assings each memory block to its output.
    genvar k,h;
    generate
        for(k = 0; k < 32; k++) begin:eachBlock
            for(h = 0; h < 64; h++) begin:eachBit
               assign out[k][h] = memory[k][h];
            end
        end
    endgenerate
	 
endmodule


module regs_testbench();
     logic [63:0]writeData; //The 64-bit data in
	 logic clk;
     logic [31:0]enable; //output from the decoder
     logic [31:0][63:0]out;

     regs dut (.writeData,.enable,.clk,.out);

	parameter CLOCK_PERIOD = 100;
	initial begin
		clk <= 0;
		forever #(CLOCK_PERIOD/2) clk <= ~clk;
	end // initial

     initial begin 
                                          @(posedge clk);
                                          @(posedge clk);
                                          @(posedge clk);
                                          @(posedge clk);
     writeData = 64'd15; enable = 32'b0000000000000000000000000000001;  @(posedge clk); // register 1
     writeData = 64'd20; enable = 32'b0000000000000001000000000000000; @(posedge clk); // all registers till 31
     writeData = 64'd35; enable = 32'b0000000000000001000000000100010; @(posedge clk); // register 29
     writeData = 64'd40; enable = 32'b0000001111000000000000000100010; @(posedge clk); // register 21
     writeData = 64'd28; enable = 32'b0000000000000000000000000000011; @(posedge clk); // register 18
	 $stop;
    end
endmodule

            
    
