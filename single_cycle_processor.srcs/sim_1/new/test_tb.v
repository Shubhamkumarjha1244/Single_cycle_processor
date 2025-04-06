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
        
        
//        //adder test
//        reg[31:0] inn1,inn2;
//        wire[31:0] out;
        
//        adder_32bit dut(inn1,inn2,out);
        
//        initial
//            begin
//                inn1=32'd5;
//                inn2=32'd7;
//                #5
//                inn1=32'd10;
//                inn2=32'd22;
//                #5
//                inn1=32'd244;
//                inn2=32'd366;
//                #5
//                $finish;
//            end


//        //instruction memory test
//        reg [31:0] addr;
//        wire [31:0] data;
        
//        Instruction_memory dut(addr,data);
        
//        initial
//            begin
//                addr=32'd0;
//                #10
//                addr=32'd1;
//                #10
//                addr=32'd2;
//                #10
//                addr=32'd3;
//                #10
//                addr=32'd4;
//                #10
//                addr=32'd5;
//                #10
//                $finish;
//            end
            
            
//        //data memory test
        
//        reg[31:0] addr;
//        reg[31:0] write_data;
//        reg write_enable;
//        reg clk=0;

        
//        wire [31:0] read_data;
        
//        data_memory dut(clk,write_enable,addr,write_data,read_data);
        
//        always
//            #5 clk=~clk;
        
//        initial 
//                begin
//                    #2
//                    write_enable=1'b0;
//                    addr=32'd1;
//                    #10
//                    write_enable=1'b1;
//                    addr=32'd0;
//                    write_data=32'd11;
//                    #10
//                    write_enable=1'b1;
//                    addr=32'd1;
//                    write_data=32'd21;
//                    #10
//                    write_enable=1'b1;
//                    addr=32'd2;
//                    write_data=32'd31;
//                    #10
//                    write_enable=1'b0;
//                    addr=32'b0;
//                    #10
//                    write_enable=1'b0;
//                    addr=32'b1;
//                    #10
//                    write_enable=1'b0;
//                    addr=32'd10;                   
//                    #10
//                    write_enable=1'b0;
//                    addr=32'd2;                   
//                    #10
//                    $finish;
//               end


//        //registor file test
//        reg clk=0;
//        reg write_enable;
//        reg[4:0] addr1,addr2,addr3;
//        reg[31:0] write_data;
//        wire[31:0] read_data1,read_data2;
         
//        register_file dut(clk,write_enable,addr1,addr2,addr3,write_data,read_data1,read_data2);
        
//        always
//            #5 clk=~clk;
            
//         initial
//            begin
//                write_enable=1'b1;
//                addr1=5'd0;
//                addr2=5'd1;
//                addr3=5'd2;
//                write_data=32'd5;
//                #10
//                write_enable=1'b1;
//                addr1=5'd2;
//                addr2=5'd1;
//                addr3=5'd0;
//                write_data=32'd10;
//                #10
//                write_enable=1'b1;
//                addr1=5'd2;
//                addr2=5'd0;
//                addr3=5'd1;
//                write_data=32'd20;
//                #10
//                write_enable=1'b0;
//                addr1=5'd2;
//                addr2=5'd0;
//                addr3=5'd1;
//                write_data=32'd20;
//                #10
//                write_enable=1'b0;
//                addr1=5'd2;
//                addr2=5'd1;
//                addr3=5'd2;
//                write_data=32'd20;
//                #10
//                write_enable=1'b1;
//                addr1=5'd2;
//                addr2=5'd1;
//                addr3=5'd1;
//                write_data=32'd121;
//                #10
//                write_enable=1'b0;
//                addr1=5'd1;
//                addr2=5'd0;
//                addr3=5'd2;
//                write_data=32'd121; 
//                #10
//                $finish;
//            end                       
        
        
        
        //extend block test
        reg imm_src;
        reg[11:0] inn;
        wire[31:0] out;
        
        extend_12_to_32 dut(imm_src,inn,out);
        
        initial 
            begin
                imm_src=1;
                inn=32'hFFA;
                #10
                inn=32'h0FD;
                #10
                imm_src=0;
                inn=32'hFFA;
                #10 
                $finish; 
            end             
        
        
                    
endmodule
