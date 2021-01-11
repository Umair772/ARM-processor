`timescale 1ns/10ps
module NOR_64to1(in,out);
    input logic [63:0] in;
    output logic out;
    logic [1:0]inter;

    NOR_32to1 mod0(.in(in[31:0]),.out(inter[0]));
    NOR_32to1 mod1(.in(in[63:32]),.out(inter[1]));
    and mod2(out,inter[0],inter[1]);

endmodule

module NOR_64to1_testbench();
    logic [63:0]in;
    logic out;
		
	NOR_32to1 dut (in,out);
	
    initial begin
    in = 64'd0; #1000; //out = 1;
    in = 64'd1; #1000;//out = 0;
    in = 64'd10; #1000;//out = 0;
    in = 64'd7; #1000;//out = 0;
    in = 64'd100; #1000;//out = 0;
    in = 64'd105; #1000;//out = 0;
    in = 64'd200; #1000;//out = 0;
    in = 64'd250; #1000;//out = 0;
    in = 64'd300; #1000;//out = 0;
    in = 64'd350; #1000;//out = 0;
    in = 64'd400; #1000;//out = 0;
    in = 64'd450; #1000;//out = 0;
    in = 64'd500; #1000;//out = 0;
    in = 64'd550; #1000;//out = 0;
    in = 64'd600; #1000;//out = 0;
    in = 64'd650; #1000;//out = 0;
    in = 64'd700; #1000;//out = 0;
    in = 64'd750; #1000;//out = 0;
    in = 64'd800; #1000;//out = 0;
    in = 64'd1000; #1000;//out = 0;
    //$stop;    
    end
	 
endmodule