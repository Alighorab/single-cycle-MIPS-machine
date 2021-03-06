`include "mips_defines.vh"
`include "internal_defines.vh"
////
//// mips_control: generate all control signals.
////
//// reg_dst (output) - Choose between rd and rt.
//// alu_src (output) - alu_op2 = rt_data or extended immediate?
//// mem_to_reg (output) - rd_data (aka: write back data) = mem_out or alu_out?
//// jr (output) - jump register unconditionally?
//// jump (output) - jump unconditionally?
//// branch (output) - branch?
//// mem_we (output) - memory write enable?
//// rd_we (output) - register file write enable?
//// alu_sel (output) - select ALU operation.
//// op (input)  - opcode.
//// funct (input)
//// rt_num (input)
////
module mips_control (
    // outputs
    reg_dst, alu_src, mem_to_reg, jr, jump, branch, mem_we, rd_we, alu_sel,
    // inputs
    op, funct, rt_num
);
    // outputs
    output reg    	reg_dst, alu_src, mem_to_reg, jr, jump, branch, mem_we, rd_we;
    output reg  [3:0]  	alu_sel;
    input  	[5:0]  	op, funct;
    input       [4:0]  	rt_num;
    always_comb begin
        if (op == 6'h0) begin
            {reg_dst, alu_src, mem_to_reg, jr, jump, branch} = 
                  (op == `OP0_JR || op == `OP0_JALR)? 6'b1xx100 : 6'b10x000;
            {mem_we, rd_we} = 
                  (op == `OP0_JR)? 2'b00 : 2'b01;

            case (funct)
                `OP0_SLL:         alu_sel <= `ALU_SLL;
                `OP0_SRL:         alu_sel <= `ALU_SRL;
                `OP0_SRA:         alu_sel <= `ALU_SRA;
                `OP0_SLLV:        alu_sel <= `ALU_SLL;
                `OP0_SRLV:        alu_sel <= `ALU_SRL;
                `OP0_ADD:         alu_sel <= `ALU_ADD;
                `OP0_ADDU:        alu_sel <= `ALU_ADD;
                `OP0_SUB:         alu_sel <= `ALU_SUB;
                `OP0_SUBU:        alu_sel <= `ALU_SUB;
                `OP0_AND:         alu_sel <= `ALU_AND;
                `OP0_OR:          alu_sel <= `ALU_OR;
                `OP0_XOR:         alu_sel <= `ALU_XOR;
                `OP0_NOR:         alu_sel <= `ALU_NOR;
                `OP0_SLT:         alu_sel <= `ALU_LT;
                `OP0_SLTU:        alu_sel <= `ALU_LT;
                default:          alu_sel <= 4'hf;
            endcase
        end
        else if (op == 6'h1) begin
            {reg_dst, alu_src, mem_to_reg, jr, jump, branch} = 6'bx1x001;
            {mem_we, rd_we} = 2'b00;

            case (rt_num) 
                `OP1_BLTZ:        alu_sel <= `ALU_LT;
                `OP1_BGEZ:        alu_sel <= `ALU_GE;
                `OP1_BLTZAL:      alu_sel <= `ALU_LT;
                `OP1_BGEZAL:      alu_sel <= `ALU_GE; 
                default:          alu_sel <= 4'hf;
            endcase
        end

        else begin
            if(op == 6'h2 || op == 6'h3) begin
                {reg_dst, alu_src, mem_to_reg, jr, jump, branch} = 6'bxxx010;
                {mem_we, rd_we} = (op == 6'h2 ? 2'b00 : 2'b01);
            end 
            else if(op >= 6'h4 && op <= 6'h7) begin
                {reg_dst, alu_src, mem_to_reg, jr, jump, branch} = 6'b01x001;
                {mem_we, rd_we} = 2'b00;
            end
            else if(op >= 6'h8 && op <= 6'he) begin
                {reg_dst, alu_src, mem_to_reg, jr, jump, branch} = 6'b01x000;
                {mem_we, rd_we} = 2'b01;
            end
            else if(op == 6'hf) begin
                {reg_dst, alu_src, mem_to_reg, jr, jump, branch} = 6'b010000;
                {mem_we, rd_we} = 2'b01;
            end
            else if(op >= 6'h20 && op <= 6'h26) begin
                {reg_dst, alu_src, mem_to_reg, jr, jump, branch} = 6'b011000;
                {mem_we, rd_we} = 2'b01;
            end
            else if(op >= 6'h28 && op <= 6'h2b) begin
                {reg_dst, alu_src, mem_to_reg, jr, jump, branch} = 6'b011000;
                {mem_we, rd_we} = 2'b10;
            end

            case (op)  
                `OP_BEQ:            alu_sel <= `ALU_SUB;  
                `OP_BNE:            alu_sel <= `ALU_SUB;
                `OP_BLEZ:           alu_sel <= `ALU_LE;
                `OP_BGTZ:           alu_sel <= `ALU_GT; 
                `OP_ADDI:           alu_sel <= `ALU_ADD;
                `OP_ADDIU:          alu_sel <= `ALU_ADD;
                `OP_SLTI:           alu_sel <= `ALU_LT;
                `OP_SLTIU:          alu_sel <= `ALU_LT;
                `OP_ANDI:           alu_sel <= `ALU_AND;
                `OP_ORI:            alu_sel <= `ALU_OR;
                `OP_XORI:           alu_sel <= `ALU_XOR;
                `OP_LUI:            alu_sel <= `ALU_ADD;
                `OP_LB:             alu_sel <= `ALU_ADD;
                `OP_LH:             alu_sel <= `ALU_ADD;
                `OP_LW:             alu_sel <= `ALU_ADD;
                `OP_LBU:            alu_sel <= `ALU_ADD;
                `OP_LHU:            alu_sel <= `ALU_ADD;
                `OP_SB:             alu_sel <= `ALU_ADD;
                `OP_SH:             alu_sel <= `ALU_ADD;
                `OP_SW:             alu_sel <= `ALU_ADD;
                default:            alu_sel <= 4'hf;
            endcase
        end
    end
endmodule