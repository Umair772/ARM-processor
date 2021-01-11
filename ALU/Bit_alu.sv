`timescale 1 ns / 10 ps
module Bit_alu(a,b,out,Cin,Cout,en);
    input logic [2:0] en;
    input logic a,b,Cin;
    output logic out,Cout;
    logic [7:0] result;
    logic bCompliment,bOut,seprate;

    // result = B for cntrl: 000
    assign result[0] = b;
    //
    fullAdder add_subtract (.a,.b,.Sum(seprate),.Cin,.Cout,.selAB(en[0]));
    //mux8_1 mod2 (.in(result[7:0]),.sel(en[2:0]),.out);
    assign result[1] = 0;
    assign result[2] = seprate;
    assign result[3] = seprate;
    assign result[7] = 0;

 //Using basic gates to do operations
    or  #50 or1 (result[5],b,a); // or gate
    and #50 and1 (result[4],b,a); // and gate
    xor #50 xor1 (result[6],b,a); // xor gate

   //Least or most significant bit first?
    mux8_1 mod2 (.in(result[7:0]),.sel(en[2:0]),.out);





endmodule 

module Bit_alu_testbench();
    logic [2:0] en;
    logic a,b,Cin;
    logic out,Cout;
    logic [7:0] result;
    logic bOut;
    logic bCompliment;
    logic seprate;

    Bit_alu dut (.a,.b,.out,.Cin,.Cout,.en);

    initial begin
    a = 0; b = 0; Cin = 0;en = 3'b010; #1000; // S = 0, Cout = 0
    a = 0; b = 0; Cin = 1;en = 3'b010; #1000; // S = 1, Cout = 0
    a = 0; b = 1; Cin = 0;en = 3'b010; #1000; // S = 1, Cout = 0
    a = 0; b = 1; Cin = 1;en = 3'b010; #1000; // S = 0, Cout = 1
    a = 1; b = 0; Cin = 0;en = 3'b010; #1000; // S = 1, Cout = 0
    a = 1; b = 0; Cin = 1;en = 3'b010; #1000; // S = 0, Cout = 1
    a = 1; b = 1; Cin = 0;en = 3'b010; #1000; // S = 0, Cout = 1
    a = 1; b = 1; Cin = 1;en = 3'b010; #1000; // S = 1, Cout = 1
                          #1000
    $stop;  
    end
	 
endmodule
