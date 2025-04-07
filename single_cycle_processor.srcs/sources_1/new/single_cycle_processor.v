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



module main_decoder(opcode,branch,result_src,mem_write,alu_src,imm_src,reg_write,alu_op); //part of control unit
        input[6:0] opcode;
        output reg branch,result_src,mem_write,alu_src,reg_write;
        output reg[1:0] imm_src,alu_op;
        
        always @(opcode)
            case (opcode)
                7'b0000011:
                        begin
                        reg_write=1'b1;
                        imm_src=2'b00;
                        alu_src=1'b1;
                        mem_write=1'b0;
                        result_src=1'b1;
                        branch=1'b0;
                        alu_op=2'b00;
                        end
                7'b0100011:
                        begin
                        reg_write=1'b0;
                        imm_src=2'b01;
                        alu_src=1'b1;
                        mem_write=1'b1;
                        result_src=1'bx;
                        branch=1'b0;
                        alu_op=2'b00;
                        end
                7'b0110011:
                        begin
                        reg_write=1'b1;
                        imm_src=2'bxx;
                        alu_src=1'b0;
                        mem_write=1'b0;
                        result_src=1'b0;
                        branch=1'b0;
                        alu_op=2'b10;
                        end
                7'b1100011:
                        begin
                        reg_write=1'b0;
                        imm_src=2'b10;
                        alu_src=1'b0;
                        mem_write=1'b0;
                        result_src=1'bx;
                        branch=1'b0;
                        alu_op=2'b01;
                        end
                endcase
               
endmodule

                
module alu_decoder(alu_op,opcode_5,func3,func7,alu_control); //part of control unit
    input[1:0] alu_op;
    input opcode_5;
    input[2:0] func3;
    input func7;
    
    output reg[2:0] alu_control;
    
    always @(alu_op,func3,func7)
        case (alu_op)
                2'b00:
                        alu_control=3'b000;
                2'b01:
                        alu_control=3'b001;
                2'b10:
                           case({func3,opcode_5,func7})
                                    5'b00000:alu_control=3'b000;
                                    5'b00001:alu_control=3'b000;
                                    5'b00010:alu_control=3'b000;
                                    5'b00011:alu_control=3'b001;
                                    5'b010xx:alu_control=3'b101;
                                    5'b110xx:alu_control=3'b011;
                                    5'b111xx:alu_control=3'b010;
                           endcase
        endcase
        
endmodule


module control_unit(zero,opcode,func3,func7,pc_src,result_src,mem_write,alu_src,imm_src,reg_write,alu_control); //control unit
    input zero;
    input[6:0] opcode;
    input[2:0] func3;
    input func7;
    output pc_src,result_src,mem_write,alu_src,reg_write;
    output[1:0] imm_src;
    output[2:0] alu_control;
    
    wire[1:0] alu_op;
    wire branch;
    
    main_decoder maindec(opcode,branch,result_src,mem_write,alu_src,imm_src,reg_write,alu_op);
    alu_decoder aludecoder(alu_op,opcode[5],func3,func7,alu_control);
    assign pc_src=zero&branch;
        
endmodule

//carry select adder substractor

module full_adder(inn1,inn2,carry_in,sum,carry_out);
    input inn1,inn2,carry_in;    //taken 2 input & carry_in wire for fulladder input
    output sum,carry_out; //taken output of sum,carry 
    
    assign sum=inn1^inn2^carry_in; //use concatenation operator for sum& carry_out generation
    assign carry_out=(inn1 & inn2)|(inn2 & carry_in)|(inn1 & carry_in);
endmodule


module two_one_mux(inn1,inn2,sel,out);
    input inn1,inn2,sel; //input for mux & select line
    output out; //output for mux
    assign out=sel?inn2:inn1; //use terninary operator for mux operation
endmodule


module one_bit_carry_select_adder(inn1,inn2,carry_in,sum,carry_out);
    input inn1,inn2,carry_in;
    output sum,carry_out;
    wire internal_sum1,internal_sum2,internal_carry1,internal_carry2;  //for internal sum & carry generated by full adder module
    
    full_adder full_adder_1(inn1,inn2,1'b0,internal_sum1,internal_carry1); //imported full adder module & connect it
    full_adder full_adder_2(inn1,inn2,1'b1,internal_sum2,internal_carry2);//imported full adder module & connect it
    
    two_one_mux mux1(internal_carry1,internal_carry2,carry_in,carry_out);//imported mux module & connect it
    two_one_mux mux2(internal_sum1,internal_sum2,carry_in,sum);//imported mux module & connect it
    
endmodule


module n_bit_carry_select_adder(innA,innB,control,sum);
    parameter   number_of_adder=32;
    input [number_of_adder-1:0] innA,innB;
    input control;
    output[number_of_adder-1:0] sum;
    
    wire [number_of_adder-1:0] xor_inn2;
    wire[number_of_adder:0] int_carry;  //internal_carry function
    
    assign int_carry[0]=control;
    
    

    genvar i;


    generate    for(i=0;i<number_of_adder;i=i+1)
        begin: byte_adder //name of block
            //initialization of 32 1B carry select Adder using generate
            assign xor_inn2[i]=innB[i]^control;     //2'S complement block
            one_bit_carry_select_adder add(innA[i],xor_inn2[i],int_carry[i],sum[i],int_carry[i+1]); //import 1 bit carry       
        end  
    endgenerate     
endmodule

//bitwise_and_block

module and_block(innA,innB,out);
    input[31:0] innA,innB;
    output[31:0] out;
    
    assign out=innA&innB;
 
endmodule

//bitwise_or_block
module or_block(innA,innB,out);
    input[31:0] innA,innB;
    output[31:0] out;
    
    assign out=innA|innB;
 
endmodule

//alu block
module alu(alu_control,innA,innB,zero,out);
    input[2:0] alu_control;
    input[31:0] innA,innB;
    output reg [31:0] out;
    output zero;
    
    wire slt;
    wire[31:0] out_adder,out_substractor,out_and,out_or;
    
    n_bit_carry_select_adder adder1(innA,innB,1'b0,out_adder);
    n_bit_carry_select_adder substractor1(innA,innB,1'b1,out_substractor);
    and_block and_1(innA,innB,out_and);
    or_block or_1(innA,innB,out_or);
    
    assign slt={{31'b0},{~out_substractor[31]}};
    assign zero=~|(out_substractor);
    
    always @(*)
        case(alu_control)
            3'b000:
                out=out_adder;
            3'b001:
                out=out_substractor;
            3'b010:
                out=out_and;
            3'b011:
                out=out_or;
            3'b101:
                out=slt;
         endcase
endmodule
                
module single_cycle_processor();

endmodule
