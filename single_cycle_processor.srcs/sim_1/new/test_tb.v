`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: IIT Patna
// Engineer: Shubham Kumar Jha
// 
// Create Date: 06.04.2025 07:24:17
// Design Name: Risc5 based single cycle processor
// Module Name: single_cycle_processor
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module test_tb();

//    //mux test
//    reg[31:0] inn1=32'd5;
//    reg[31:0] inn2=32'd10;
//    reg sel;
//    wire[31:0] out;
    
//    two_one_mux_32_bit dut(inn1,inn2,sel,out);
    
//    initial
//        begin
//            sel=1'b0;
//            #10
//            sel=1'b1;
//            #10
//            $finish;
//        end




//      //d_ff test 
//      reg[31:0] inn;
//      reg clk=1'b0;
//      wire[31:0] out;
      
//      d_ff_32bit dut(clk,inn,out);
      
//      always
//        #5 clk=~clk;
        
//      initial
//        begin
//        #2
//        inn=32'd1;
//        #10
//        inn=32'd2;
//        #10
//        inn=32'd3;
//        #10
//        inn=32'd4;
//        #10
//        inn=32'd5;
//        $finish;
//        end    
        
        
        //adder test
        reg[31:0] inn1,inn2;
        wire[31:0] out;
        
        adder_32bit dut(inn1,inn2,out);
        
        initial
            begin
                inn1=32'd5;
                inn2=32'd7;
                #5
                inn1=32'd10;
                inn2=32'd22;
                #5
                inn1=32'd244;
                inn2=32'd366;
                #5
                $finish;
            end
                
        
      

    

    
    
endmodule
