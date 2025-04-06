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
        out<=inn;

endmodule   //tested

module adder_32bit(inn1,inn2,out); //adding 2 32bit number
    input[31:0] inn1,inn2;
    output[31:0] out;
    
    assign out=inn1+inn2;
    
endmodule

module Instruction_memory(addr,out_data); //instruction memory as rom with 32bit address line for 32bit word
    input[31:0] addr;
    output[31:0] out_data;
    
    reg[31:0] inst_mem[31:0];
      
    assign out_data=inst_mem[addr];

endmodule //tested

module data_memory(clk,write_enable,address,write_data,read_data); //data memory as a single port ram with 32 bit address line for 32bit data word
    input clk,write_enable;
    input[31:0] address;
    input[31:0] write_data;
    output reg [31:0] read_data;
    
    reg[31:0] data_mem[31:0];
    
    always @(posedge clk)
        begin
            if(write_enable)
                data_mem[address]<=write_data;
            else
                read_data<=data_mem[address];
        end
                
endmodule //tested

module register_file(clk,write_enable,addr1,addr2,addr3,write_data,read_data1,read_data2); //3 addr register file
    input clk;
    input write_enable;
    input[4:0] addr1,addr2,addr3;
    input[31:0] write_data;
    output reg [31:0] read_data1;
    output reg  [31:0] read_data2;
    
    reg[31:0] register_file[4:0];
    
    always @(posedge clk)
        begin
        read_data1<=register_file[addr1];
        read_data2<=register_file[addr2];
        if(write_enable)
                register_file[addr3]<=write_data;
        end
endmodule //tested


module extend_12_to_32(imm_src,inn,out); //use to extend 12bit to 32bit data
    input[11:0] inn;
    input imm_src;
    output reg[31:0] out;
    
    always @(*)
        if(imm_src)
            begin
            out[11:0]=inn;
            out[31:12]={5'd20{inn[11]}};
            end
endmodule //tested


    
    
    


            
        
    
    




























module single_cycle_processor();
endmodule
