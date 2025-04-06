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

module two_one_mux_32_bit(inn1,inn2,sel,out); //mux for forwarding 32bit data 
    input[31:0] inn1,inn2;
    input sel;
    input[31:0] out;
    
    assign out=sel?inn2:inn1;
    
 endmodule //tested


module d_ff_32bit(clk,inn,out); //dff for clock syncronization of single cycle processor
    input[31:0] inn;
    input clk;
    output reg[31:0] out;
    
    always @(posedge clk)
        out=inn;

endmodule   //tested

module adder_32bit(inn1,inn2,out); //adding 2 32bit number
    input[31:0] inn1,inn2;
    output[31:0] out;
    
    assign out=inn1+inn2;
    
endmodule




























module single_cycle_processor();
endmodule
