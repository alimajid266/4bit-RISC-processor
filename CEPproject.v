

module program_counter (
    output reg [3:0] address,
    input  [3:0] pc_next,    //full next address
    input        upd,        //when high load pc_next
    input        clk,
    input        clr
);
    always @(posedge clk or posedge clr) begin
        if (clr)
            address <= 4'b0000;
        else if (upd)
            address <= pc_next;
    end
endmodule



module alu_4bit (
    input  [3:0] A,
    input  [3:0] B,
    input  [2:0] alu_control,  
    output reg [3:0] result,
    output reg       cf,       
    output reg       of,       
    output reg       zf        
);
    reg [4:0] temp;

    always @(*) begin
        // defaults
        cf = 1'b0;
        of = 1'b0;
        result = 4'b0000;

        case (alu_control)
            3'b000: result = A & B;
            3'b001: result = A | B;

            3'b010: begin // ADD
                temp = {1'b0, A} + {1'b0, B};
                result = temp[3:0];
                cf = temp[4];
                //overflow
                of = (A[3] & B[3] & ~result[3]) |
                     (~A[3] & ~B[3] &  result[3]);
            end

            3'b011: begin // SUB
                temp = {1'b0, A} - {1'b0, B};
                result = temp[3:0];
                cf = temp[4];
                // overflow 
                of = (A[3] & ~B[3] & ~result[3]) |
                     (~A[3] &  B[3] &  result[3]);
            end

            3'b111: result = (A < B) ? 4'b0001 : 4'b0000; // SLT

            default: result = 4'b0000;
        endcase

        zf = (result == 4'b0000);  //zero flag
    end
endmodule



module decoder2x4(
    input [1:0] sel,
    input en,
    output reg [3:0] out
);
always @(*) begin
    if (en) begin
        case (sel)
            2'b00: out = 4'b0001;
            2'b01: out = 4'b0010;
            2'b10: out = 4'b0100;
            2'b11: out = 4'b1000;
            default: out = 4'b0000;
        endcase
    end else
        out = 4'b0000;
end
endmodule

module register4bit(
    input [3:0] d,
    input clk,
    input reset,
    input load,
    output reg [3:0] q
);
always @(posedge clk) begin
    if (reset)
        q <= 4'b0000;
    else if (load)
        q <= d;
end
endmodule

module mux4to1(
    input [3:0] in0,
    input [3:0] in1,
    input [3:0] in2,
    input [3:0] in3,
    input [1:0] sel,
    output reg [3:0] out
);
always @(*) begin
    case (sel)
        2'b00: out = in0;
        2'b01: out = in1;
        2'b10: out = in2;
        2'b11: out = in3;
        default: out = 4'b0000;
    endcase
end
endmodule

module register_file_4x4(
    input clk,
    input reset,
    input wr_en,
    input [1:0] wr_addr,
    input [1:0] rd_addr1,
    input [1:0] rd_addr2,
    input [3:0] data_in,
    output [3:0] out1,
    output [3:0] out2
);
    wire [3:0] load;
    wire [3:0] q [3:0];

    decoder2x4 dec(
        .sel(wr_addr),
        .en(wr_en),
        .out(load)
    );

    register4bit reg0(.d(data_in), .clk(clk), .reset(reset), .load(load[0]), .q(q[0]));
    register4bit reg1(.d(data_in), .clk(clk), .reset(reset), .load(load[1]), .q(q[1]));
    register4bit reg2(.d(data_in), .clk(clk), .reset(reset), .load(load[2]), .q(q[2]));
    register4bit reg3(.d(data_in), .clk(clk), .reset(reset), .load(load[3]), .q(q[3]));

    mux4to1 mux1(.in0(q[0]), .in1(q[1]), .in2(q[2]), .in3(q[3]), .sel(rd_addr1), .out(out1));
    mux4to1 mux2(.in0(q[0]), .in1(q[1]), .in2(q[2]), .in3(q[3]), .sel(rd_addr2), .out(out2));
endmodule




module datapath (
    input clk,
    input clr,
    input pc_inc,
    input write_enable,
    input [5:0] instruction,
    input [2:0] alu_control,
    input [3:0] ext_data_in,
    input select_data_source,
    input branch,      
    output [3:0] alu_result,
    output [3:0] pc_out,
    input [3:0] imm,           
    output  carry_flag, 
    output overflow_flag,
    output  zero_flag
);
    wire [1:0] rd  = instruction[5:4];
    wire [1:0] rs1 = instruction[3:2];
    wire [1:0] rs2 = instruction[1:0];
    wire [3:0] read_data1, read_data2;
    reg [3:0] data_to_write;

    wire [3:0] alu_out;

    always @(*) begin
        if (select_data_source)
            data_to_write = ext_data_in;
        else
            data_to_write = alu_out;
    end
    wire [3:0] pc_plus1      = pc_out + 4'd1;
    wire [3:0] branch_target = pc_plus1 + imm; 

    wire [3:0] pc_next = branch     ? branch_target :
                          pc_inc     ? pc_plus1     :
                                       pc_out;

    program_counter PC (
        .address(pc_out),
        .pc_next(pc_next),
	.upd(branch|pc_inc),
        .clk(clk),
        .clr(clr)
    );

    register_file_4x4 RF (
        .clk(clk),
        .reset(clr),
        .wr_en(write_enable),
        .wr_addr(rd),
        .rd_addr1(rs1),
        .rd_addr2(rs2),
        .data_in(data_to_write),
        .out1(read_data1),
        .out2(read_data2)
    );

    alu_4bit ALU (
        .A(read_data1),
        .B(read_data2),
        .alu_control(alu_control),
        .result(alu_out),
	.cf(carry_flag),
	.of(overflow_flag),
	.zf(zero_flag)
    );

    assign alu_result = alu_out;

endmodule



module control_unit (
    input clk,   
    input rst,           
    input [2:0] opcode,               
    input zero_flag,
    output reg [2:0] alu_control,  
    output reg write_enable,            
    output reg select_data_source,     
    output reg pc_inc,   
    output reg branch,               
    output reg clr,                      
    output reg [1:0] stateprint 
);

    
    reg [1:0] current_state, next_state;
    localparam RESET   = 2'b00,
	       FETCH   = 2'b01,
               DECODE  = 2'b10,
               EXECUTE = 2'b11;

localparam     BEQ = 3'b011,
               BNE = 3'b100;



 
    always @(*) begin
	branch = 0;
        alu_control = 3'b000;
        write_enable = 0;
        select_data_source = 0;    
        pc_inc = 0;
        clr = 0;

        case (current_state)
            RESET: begin      //clearing 
                clr = 1;
            end
            FETCH: begin   
                clr = 0;
                pc_inc = 0;
                write_enable = 0;
            end
            DECODE: begin
                alu_control = opcode; 
            end
            EXECUTE: begin
		case(opcode)
                BEQ: begin
		if (zero_flag) branch = 1;
		else pc_inc = 1;
		end
		
                BNE: begin 
		 if (!zero_flag) branch = 1;
		else pc_inc = 1;
		end
		
		default:begin
                write_enable = 1;
                select_data_source = 0;
                pc_inc = 1;
		
		end
		endcase
            end
        endcase
    end

    // to get the next state
    always @(*) begin
        case (current_state)
            RESET:    next_state = FETCH;
            FETCH:    next_state = DECODE;
            DECODE:   next_state = EXECUTE;
            EXECUTE:  next_state = FETCH;
            default:  next_state = RESET;
        endcase
    end

 
    always @(posedge clk or posedge rst) begin
        if (rst)
            current_state <= RESET; 
        else
            current_state <= next_state; 
        stateprint <= current_state;

    end
endmodule





module inst_mem(
input [7:0] addr, 
output reg [7:0] instruction);

reg [7:0] memory [0:15];
integer i;

initial begin
for(i = 0; i < 16 ; i = i + 1)
begin
	memory[i] = 8'b00000000;
end
memory[0] = 8'b00000001;   //instruction 1 
memory[1] = 8'b00000010;
memory[2] = 8'b00000011;
memory[3] = 8'b00000100;
memory[4] = 8'b00000101;   //for testing purposes
memory[5] = 8'b00000110;
memory[6] = 8'b00000111;
memory[7] = 8'b00001000;
memory[8] = 8'b00001001;
memory[9] = 8'b00001010;
end
always @(addr) begin
	if(addr < 16)
	instruction = memory[addr];
	else
	instruction = 8'b00000000;
end

endmodule


module cpu13(
    input wire clk,
    input wire clr,               
    input wire pc_inc,           
    input wire write_enable,     
    input wire [5:0] instruction,       
    input wire [2:0] alu_control,       
    input wire [3:0] ext_data_in,       
    input wire select_data_source,
    input wire branch,           
    input wire [3:0] imm,               
    output wire [3:0] alu_result,       
    output wire [3:0] pc_out,           
    output wire carry_flag,       
    output wire overflow_flag,     
    output wire zero_flag         
);

    datapath DP (
        .clk (clk),
        .clr (clr),
        .pc_inc  (pc_inc),
        .write_enable    (write_enable),
        .instruction  (instruction),
        .alu_control  (alu_control),
        .ext_data_in  (ext_data_in),
        .select_data_source (select_data_source),
        .branch (branch),
        .imm  (imm),
        .alu_result  (alu_result),
        .pc_out   (pc_out),
        .carry_flag     (carry_flag),
        .overflow_flag   (overflow_flag),
        .zero_flag    (zero_flag)
    );

endmodule


 


module tb_cpu13;
   
    reg clk = 0;
    reg clr = 1;

    // Datapath control inputs (all initialized)
    reg pc_inc = 0;
    reg write_enable = 0;
    reg  [5:0]  instruction   = 6'b000000;
    reg  [2:0]  alu_control  = 3'b000;
    reg  [3:0]  ext_data_in  = 4'b0000;
    reg select_data_source = 0;
    reg branch   = 0;
    reg  [3:0]  imm    = 4'b0000;

    // Outputs
    wire [3:0] alu_result;
    wire [3:0] pc_out;
    wire carry_flag;
    wire overflow_flag;
    wire zero_flag;

    // Instantiate the CPU13 wrapper
    cpu13 UUT (
        .clk (clk),
        .clr  (clr),
        .pc_inc   (pc_inc),
        .write_enable (write_enable),
        .instruction   (instruction),
        .alu_control (alu_control),
        .ext_data_in  (ext_data_in),
        .select_data_source (select_data_source),
        .branch  (branch),
        .imm   (imm),
        .alu_result  (alu_result),
        .pc_out  (pc_out),
        .carry_flag  (carry_flag),
        .overflow_flag (overflow_flag),
        .zero_flag  (zero_flag)
    );

   
    always #5 clk = ~clk;

    initial begin
        // release reset
        #10  clr = 0;

        //  R0 = 4
        select_data_source = 1;
        write_enable=1;
        instruction=6'b000000;  // rd=00
        ext_data_in=4;
        #10 
        write_enable=0;
        select_data_source = 0;

        //  R1 = 3
        #10
        select_data_source = 1;            //  ext_data_in  
        write_enable = 1;
        instruction=6'b010000;  // rd=01
        ext_data_in=3;
        #10
        write_enable=0;
        select_data_source = 0;

        //  ADD R2 = R1 + R0  =    7
        #10
        alu_control=3'b010;      
        pc_inc=1;
        write_enable=1;
        instruction=6'b100100;  // rd=10, rs1=01, rs2=00
        #10
        write_enable=0;
        pc_inc=0;

          // ? after your ADD step ?

    //BEQ test: compare R2 to R2 meaning zero_flag=1 and branch taken
    @(posedge clk);
      alu_control=3'b011;   //subtraction
      imm=4;          //branch offset
      select_data_source = 0;         
      write_enable=0;        
    #1;

    @(posedge clk);
      branch =  zero_flag;            // should be 1 here
      pc_inc = !zero_flag;         
    //clearing
    @(posedge clk);
      branch = 0;
      pc_inc = 0;


    //BNE 
    @(posedge clk);
      alu_control        = 3'b011;    // SUB
      imm                = 2;         
    #1;

    @(posedge clk);
      branch = !zero_flag;            //should be 1
      pc_inc = zero_flag;             

    // clearing
    @(posedge clk);
      branch = 0;
      pc_inc = 0;
    
        //SUB  R3 = R0 - R1 = 4-3 = 1
        #10
        alu_control=3'b011;      
        write_enable=1;
        instruction=6'b110001;  
        #10
        write_enable=0;

        //  AND test: R2 =    4&3 = 0
        #10
        alu_control=3'b000;       
        write_enable=1;
        instruction=6'b100001;  
        #10
        write_enable=0;

        // OR test:  R3 =   4|3 = 7
        #10
        alu_control=3'b001;       // OR
        write_enable=1;
        instruction=6'b110001;  
        #1
        write_enable=0; 

        // 8) SLT test: R2   3<4 = 1
        #10
        alu_control=3'b111;       // SLT
        write_enable=1;
        instruction=6'b100100; 
        #10
        write_enable=0;

        #20 $finish;
    end
endmodule
