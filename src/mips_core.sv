/*
 *
 * Redistributions of any form whatsoever must retain and/or include the
 * following acknowledgment, notices and disclaimer:
 *
 * This product includes software developed by Carnegie Mellon University. 
 *
 * Copyright (c) 2004 by Babak Falsafi and James Hoe,
 * Computer Architecture Lab at Carnegie Mellon (CALCM), 
 * Carnegie Mellon University.
 *
 * This source file was written and maintained by Jared Smolens 
 * as part of the Two-Way In-Order Superscalar project for Carnegie Mellon's 
 * Introduction to Computer Architecture course, 18-447. The source file
 * is in part derived from code originally written by Herman Schmit and 
 * Diana Marculescu.
 *
 * You may not use the name "Carnegie Mellon University" or derivations 
 * thereof to endorse or promote products derived from this software.
 *
 * If you modify the software you must place a notice on or within any 
 * modified version provided or made available to any third party stating 
 * that you have modified the software.  The notice shall include at least 
 * your name, address, phone number, email address and the date and purpose 
 * of the modification.
 *
 * THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT ANY WARRANTY OF ANY KIND, EITHER 
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO ANYWARRANTY 
 * THAT THE SOFTWARE WILL CONFORM TO SPECIFICATIONS OR BE ERROR-FREE AND ANY 
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, 
 * TITLE, OR NON-INFRINGEMENT.  IN NO EVENT SHALL CARNEGIE MELLON UNIVERSITY 
 * BE LIABLE FOR ANY DAMAGES, INCLUDING BUT NOT LIMITED TO DIRECT, INDIRECT, 
 * SPECIAL OR CONSEQUENTIAL DAMAGES, ARISING OUT OF, RESULTING FROM, OR IN 
 * ANY WAY CONNECTED WITH THIS SOFTWARE (WHETHER OR NOT BASED UPON WARRANTY, 
 * CONTRACT, TORT OR OTHERWISE).
 *
 */

//////
////// MIPS 447: A single-cycle MIPS ISA simulator
//////

// Include the MIPS constants
`include "mips_defines.vh"
`include "internal_defines.vh"

////
//// The MIPS standalone processor module
////
////   clk          (input)  - The clock
////   inst_addr    (output) - Address of instruction to load
////   inst         (input)  - Instruction from memory
////   inst_excpt   (input)  - inst_addr not valid
////   mem_addr     (output) - Address of data to load
////   mem_data_in  (output) - Data for memory store
////   mem_data_out (input)  - Data from memory load
////   mem_write_en (output) - Memory write mask
////   mem_excpt    (input)  - mem_addr not valid
////   halted       (output) - Processor halted
////   reset        (input)  - Reset the processor
////   

module mips_core(/*AUTOARG*/
    // Outputs
    inst_addr, mem_addr, mem_data_in, mem_write_en, halted,
    // Inputs
    clk, inst, mem_data_out, rst_b
    );
    parameter text_start = 32'h00400000;
    // Core Interface
    input         clk;
    output [29:0] inst_addr;
    output [29:0] mem_addr;
    input  [31:0] inst, mem_data_out;
    output [31:0] mem_data_in;
    output [3:0]  mem_write_en;
    output        halted;
    input         rst_b;

    // Internal signals 
    reg  [31:0]   pc, nextpc, nextnextpc;
    wire          syscall_halt, internal_halt;
    wire [31:0]   rs_data, rt_data, rd_data, alu_out, r_v0;

    // Decode signals 
    wire [31:0]   dcd_se_imm, dcd_se_offset, dcd_e_imm, dcd_se_mem_offset;
    wire [5:0]    dcd_op, dcd_funct;
    wire [4:0]    dcd_rs, dcd_rt, dcd_rd, dcd_shamt;
    wire [15:0]   dcd_imm;
    wire [25:0]   dcd_target;

    // Control signals 
    wire          reg_dst, alu_src, mem_to_reg, jr, jump, branch;
    wire          mem_we, rd_we;
    wire [3:0]    alu_sel;
    wire          zero;

    // PC Management 
    register  #(32, text_start)     currentPC(pc, nextpc, clk, ~internal_halt, rst_b);
    register  #(32, text_start+4)   nextPC(nextpc, nextnextpc, clk, ~internal_halt, rst_b);
    always_comb begin
        if (branch && zero) begin
            nextnextpc <= nextpc + dcd_se_offset;
        end else if (jump) begin
            nextnextpc <= {6'h0, dcd_target};
        end else if (jr) begin
            nextnextpc <= rs_data;
        end else begin
            nextnextpc <= nextpc + 4; 
        end
    end
         
    assign inst_addr = pc[31:2];

    // Instruction Decoding
    assign dcd_op             = inst[31:26];
    assign dcd_rs             = inst[25:21];
    assign dcd_rt             = inst[20:16];
    assign dcd_rd             = inst[15:11];
    assign dcd_shamt          = inst[10:6];
    assign dcd_funct          = inst[5:0];
    assign dcd_imm            = inst[15:0];
      // Sign-extended offset for branches
    assign dcd_se_offset      = { {14{dcd_imm[15]}}, dcd_imm, 2'b00 };
      // Sign-extended immediate for load/store and arithmetic operations
    assign dcd_se_imm  = { {16{dcd_imm[15]}}, dcd_imm };
      // Zero-extended immediate
    assign dcd_e_imm          = { 16'h0, dcd_imm };
    assign dcd_target         = inst[25:0];

    // synthesis translate_off
   always @(posedge clk) begin
     // useful for debugging, you will want to comment this out for long programs
     if (rst_b) begin
       $display ( "=== Simulation Cycle %d ===", $time );
       $display ( "[pc=%x, inst=%x] [op=%x, rs=%d, rt=%d, rd=%d, imm=%x, funct=%x] [reset=%d, halted=%d]",
                   pc, inst, dcd_op, dcd_rs, dcd_rt, dcd_rd, dcd_imm, dcd_funct, ~rst_b, halted);
     end
   end
   // synthesis translate_on

    // Generate control signals
    mips_control CU(reg_dst, alu_src, mem_to_reg, jr, jump, branch,
                              mem_we, rd_we, alu_sel,
                              dcd_op, dcd_funct, rt_num); 

    // Register file
    regfile register_file(rs_data, rt_data,
                          rs_num, rt_num, (reg_dst? rd_num : rt_num), 
                          (mem_to_reg? mem_data_out : alu_out),
                          rd_we, clk, rst_b, halted);

    // Execute
    mips_ALU ALU(alu_out, zero,
                 alu_op1, alu_op2, alu_sel,
                 dcd_shamt);

    // Syscall unit
    assign r_v0 = rs_data;
    syscall_unit SU(syscall_halt, 
                    pc, r_v0, (op == 6'h0 && dcd_funct == `OP0_SYSCALL), rst_b, clk);

endmodule // mips_core

////
//// mips_ALU: Performs all arithmetic and logical operations
////
//// out (output) - Final result
//// in1 (input)  - Operand modified by the operation
//// in2 (input)  - Operand used (in arithmetic ops) to modify in1
//// sel (input)  - Selects which operation is to be performed
////
module mips_ALU(
   // outputs
   alu_out, zero,
   // inputs
   alu_op1, alu_op2, alu_sel, shamt
  );

	output reg 		[31:0]	alu_out;
	output 					zero;
	input 			[31:0]	alu_op1, alu_op2;
	input 			[3:0]	alu_sel;
	input 			[4:0]	shamt;

	assign zero = (alu_out == 0); // zero signal is true if the alu_out = 0.
	always_comb begin
		case (alu_sel)
		`ALU_SLL:	alu_out <= alu_op2 << shamt;
		`ALU_SRL:	alu_out <= alu_op2 >> shamt;
		`ALU_SRA:	alu_out <= alu_op2 >>> shamt;
		`ALU_ADD:	alu_out <= alu_op1 + alu_op2;
		`ALU_SUB:	alu_out <= alu_op1 + alu_op2;
		`ALU_AND:	alu_out <= alu_op1 & alu_op2;
		`ALU_OR:	alu_out <= alu_op1 | alu_op2;
		`ALU_XOR:	alu_out <= alu_op1 ^ alu_op2;
		`ALU_NOR:	alu_out <= ~(alu_op1 | alu_op2);
		`ALU_LT:	alu_out <= (alu_op1 <  alu_op2 ? 1 : 0);
		`ALU_LE:	alu_out <= (alu_op1 <= alu_op2 ? 1 : 0);
		`ALU_GT:	alu_out <= (alu_op1 >  alu_op2 ? 1 : 0);
		`ALU_GE:	alu_out <= (alu_op1 >= alu_op2 ? 1 : 0);
		default:	alu_out <= 32'd0;
		endcase
	end
endmodule

//// register: A register which may be reset to an arbirary value
////
//// q      (output) - Current value of register
//// d      (input)  - Next value of register
//// clk    (input)  - Clock (positive edge-sensitive)
//// enable (input)  - Load new value?
//// reset  (input)  - System reset
////
module register(
   // output
   q, 
   // input
   d, clk, enable, rst_b);

   parameter
            width = 32,
            reset_value = 0;

   output [(width-1):0] q;
   reg [(width-1):0]    q;
   input [(width-1):0]  d;
   input                 clk, enable, rst_b;

   always @(posedge clk or negedge rst_b)
     if (~rst_b)
       q <= reset_value;
     else if (enable)
       q <= d;

endmodule // register

////
//// adder
////
//// out (output) - adder result
//// in1 (input)  - Operand1
//// in2 (input)  - Operand2
//// sub (input)  - Subtract?
////
module adder(
   // output
   out, 
   // input
   in1, in2);

   output [31:0]  out;
   input  [31:0]  in1, in2;

   assign        out = in1 + in2;

endmodule // adder

// Local Variables:
// verilog-library-directories:("." "../447rtl")
// End: