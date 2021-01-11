`timescale 1 ns / 10 ps
module fullAdder(a,b,selAB,Sum,Cin,Cout);
    input logic a,b,Cin,selAB;
    output logic Sum,Cout;
    wire [2:0]w;

    // full adder/subtractor
    not #50 inv(bCompliment,b);
    
    //Use a mux to determine if we are either adding or subtracting
     mux_2_1 m2(.in({bCompliment,b}), .sel(selAB),.out(bOut));

     xor #50 basicGate0(w[0],a,bOut);
     and #50 basicGate1(w[1],a,bOut);
     and #50 basicGate2(w[2],w[0],Cin);
     or #50 basicGate3(Cout,w[1],w[2]);
     xor #50 basicGate4(Sum,w[0],Cin);

endmodule

module fullAdder_testbench();
 logic  a,b,Cin;
 logic Sum,Cout;

 fullAdder dut (.a,.b,.Sum,.Cin,.Cout);
 
 initial begin
    a = 0; b = 0; Cin = 0; #1000; // S = 0, Cout = 0
    a = 0; b = 0; Cin = 1; #1000; // S = 1, Cout = 0
    a = 0; b = 1; Cin = 0; #1000; // S = 1, Cout = 0
    a = 0; b = 1; Cin = 1; #1000; // S = 0, Cout = 1
    a = 1; b = 0; Cin = 0; #1000; // S = 1, Cout = 0
    a = 1; b = 0; Cin = 1; #1000; // S = 0, Cout = 1
    a = 1; b = 1; Cin = 0; #1000; // S = 0, Cout = 1
    a = 1; b = 1; Cin = 1; #1000; // S = 1, Cout = 1
                           #1000
$stop;
end
endmodule







