`timescale 1ns / 1ps

module pipelinedProcessor(
    input clk,
    input reset,
    output reg [63:0] PC_In, PC_Out, ReadData1, ReadData2, WriteData, Result, Read_Data, imm_data,
    output reg [31:0] Instruction,
    output reg [6:0] opcode,
    output reg [4:0] rs1, rs2, rd,
    output reg [1:0] ALUOp,
    output reg [63:0] adder_out1, adder_out2,
    output reg Branch, MemRead, MemWrite, MemtoReg, ALUSrc, RegWrite, Zero,
    output reg [63:0] index0, index1, index2, index3, index4
    );
    //normal wires
    
    wire [63:0] PC_In, PC_Out, adder_out1, adder_out2, imm_data, WriteData, ReadData1, ReadData2, Result, Read_Data;
    wire [63:0] muxmid_out;
    wire [31:0] Instruction;
    wire [6:0] opcode, funct7;
    wire [4:0] rd, rs1, rs2;
    wire [3:0] Funct, Operation;
    wire [2:0] funct3;
    wire [1:0] ALUOp;
    wire Branch, MemRead, MemWrite, MemtoReg, ALUSrc, RegWrite, Zero, sel_Branch;
    wire [63:0] index0, index1, index2, index3, index4;
    
    //wires for IF_ID
    
    wire [31:0] IF_ID_Instruction;
    wire [63:0] IF_ID_PC_Out;
    
    //wires for ID_EX
    
    wire ID_EX_Branch, ID_EX_MemRead, ID_EX_MemWrite, ID_EX_MemtoReg, ID_EX_ALUSrc, ID_EX_RegWrite;
    wire [1:0] ID_EX_ALUOp;
    wire [63:0] ID_EX_PC_Out, ID_EX_ReadData1, ID_EX_ReadData2, ID_EX_Imm_Data;
    wire [4:0] ID_EX_RS1, ID_EX_RS2, ID_EX_RD;
    wire [3:0] ID_EX_Funct;
    wire [2:0] ID_EX_Funct3;
    
    //wire for EX_MEM
    
    wire EX_MEM_Branch, EX_MEM_Zero, EX_MEM_MemRead, EX_MEM_MemWrite, EX_MEM_MemtoReg, EX_MEM_RegWrite;
    wire [63:0] EX_MEM_Adder_Out_2, EX_MEM_Result, EX_MEM_Write_Data;
    wire [4:0] EX_MEM_RD;
    wire [1:0] ForwardA, ForwardB;
    
    //wire for MEM_WB
    
    wire MEM_WB_MemtoReg, MEM_WB_RegWrite;
    wire [63:0] MEM_WB_Read_Data, MEM_WB_Result;
    wire [4:0] MEM_WB_RD;
    
    // miscellaneous wires
    wire [63:0] mux_ReadData1, mux_ReadData2;
    
    // Instruction Fetch (IF) Modules
    
    Adder A1(.A(PC_Out), .B(64'd4), .Out(adder_out1));
    Mux_2x1 muxfirst(.A(adder_out1), .B(adder_out2), .S(sel_Branch && ID_EX_Branch), .Out(PC_In));
    Program_Counter PC(.clk(clk), .reset(reset), .PC_In(PC_In), .PC_Out(PC_Out));
    Instruction_Memory IM(.Inst_Address(PC_Out), .Instruction(Instruction));
    
    //IF/ID Pipeline Register Module
    IF_ID IFID(
    .clk(clk), .reset(reset), .PC_Out(PC_Out), .Instruction(Instruction), .PC_Out_IF_ID(IF_ID_PC_Out),
    .Instruction_IF_ID(IF_ID_Instruction));
    
    // Instruction Decode (ID) Modules / Register File Read
    Instruction_Parser IP(.Instruction(IF_ID_Instruction), .Opcode(opcode), .RD(rd), .Funct3(funct3), 
    .RS1(rs1), .RS2(rs2), .Funct7(funct7));
    Imm_Gen Immgen(.Instruction(IF_ID_Instruction), .Imm(imm_data));
    Control_Unit cu(.Opcode(opcode), .Branch(Branch), .MemRead(MemRead), .MemtoReg(MemtoReg), .ALUOp(ALUOp), 
    .MemWrite(MemWrite), .ALUSrc(ALUSrc), .RegWrite(RegWrite));
    RegisterFile rf(.clk(clk), .reset(reset), .WriteData(WriteData), .RS1(rs1), .RS2(rs2), .RD(MEM_WB_RD), 
    .RegWrite(MEM_WB_RegWrite), .ReadData1(ReadData1), .ReadData2(ReadData2));
    assign Funct = {Instruction[30], Instruction[14:12]};
    
    //ID/EX Pipeline Register Module
    ID_EX IDEX(.clk(clk), .reset(reset), .Branch(Branch), .MemRead(MemRead), .MemWrite(MemWrite), 
    .MemtoReg(MemtoReg), .ALUSrc(ALUSrc), .RegWrite(RegWrite), .ALUOp(ALUOp), .PC_Out(IF_ID_PC_Out), 
    .ReadData1(ReadData1), .ReadData2(ReadData2), .Imm_Data(imm_data), .RS1(rs1), .RS2(rs2), .RD(rd), 
    .Funct(Funct), .Funct3(funct3), .ID_EX_Branch(ID_EX_Branch), .ID_EX_MemRead(ID_EX_MemRead), 
    .ID_EX_MemWrite(ID_EX_MemWrite), .ID_EX_MemtoReg(ID_EX_MemtoReg), .ID_EX_ALUSrc(ID_EX_ALUSrc), 
    .ID_EX_RegWrite(ID_EX_RegWrite), .ID_EX_ALUOp(ID_EX_ALUOp), .ID_EX_PC_Out(ID_EX_PC_Out), 
    .ID_EX_ReadData1(ID_EX_ReadData1), .ID_EX_ReadData2(ID_EX_ReadData2), .ID_EX_Imm_Data(ID_EX_Imm_Data),
    .ID_EX_RS1(ID_EX_RS1), .ID_EX_RS2(ID_EX_RS2), .ID_EX_RD(ID_EX_RD),.ID_EX_Funct(ID_EX_Funct), .ID_EX_Funct3(ID_EX_Funct3));
    
    // Execute (EX) / Address Calculation  
    Adder A2(.A(ID_EX_PC_Out), .B(ID_EX_Imm_Data*2), .Out(adder_out2));
    Forwarding_Unit FU(ID_EX_RS1, ID_EX_RS2, EX_MEM_RD, MEM_WB_RD, EX_MEM_RegWrite, MEM_WB_RegWrite,
    ForwardA, ForwardB);
    Mux_3x1 Mux_3x1_A(.A(ID_EX_ReadData1), .B(WriteData), .C(EX_MEM_Result), .sel(ForwardA), .O(mux_ReadData1));
    Mux_3x1 Mux_3x1_B(.A(ID_EX_ReadData2), .B(WriteData), .C(EX_MEM_Result), .sel(ForwardB), .O(mux_ReadData2));
    Mux_2x1 muxmid(.A(mux_ReadData2), .B(ID_EX_Imm_Data), .S(ID_EX_ALUSrc), .Out(muxmid_out));
    ALU_Control aluc(.ALUOp(ID_EX_ALUOp), .Funct(ID_EX_Funct), .Operation(Operation));
    Branch_unit BU(.Funct3(ID_EX_Funct3), .ReadData1(mux_ReadData1), .ReadData2(mux_ReadData2), .addermuxselect(sel_Branch));
    ALU64bit ALU(.A(mux_ReadData1), .B(muxmid_out), .ALUOp(Operation), .Zero(Zero), .Result(Result));
    
    //EX/MEM Pipeline Register Module
    EX_MEM EXMEM(.clk(clk), .reset(reset), .Branch(ID_EX_Branch), .Zero(sel_Branch), .MemRead(ID_EX_MemRead), .MemWrite(ID_EX_MemWrite), .MemtoReg(ID_EX_MemtoReg), 
    .RegWrite(ID_EX_RegWrite), .Adder_Out_2(adder_out2), .Result(Result), .Write_Data(mux_ReadData2), .RD(ID_EX_RD),
    .EX_MEM_Branch(EX_MEM_Branch), .EX_MEM_Zero(EX_MEM_Zero), .EX_MEM_MemRead(EX_MEM_MemRead), .EX_MEM_MemWrite(EX_MEM_MemWrite), 
    .EX_MEM_MemtoReg(EX_MEM_MemtoReg), .EX_MEM_RegWrite(EX_MEM_RegWrite), .EX_MEM_Adder_Out_2(EX_MEM_Adder_Out_2), 
    .EX_MEM_Result(EX_MEM_Result), .EX_MEM_Write_Data(EX_MEM_Write_Data), .EX_MEM_RD(EX_MEM_RD));
    
    // Memory Access (MEM)
    Data_Memory DM(.clk(clk), .MemWrite(EX_MEM_MemWrite), .MemRead(EX_MEM_MemRead), .Mem_Addr(EX_MEM_Result), 
    .Write_Data(EX_MEM_Write_Data), .Read_Data(Read_Data), .index0(index0), .index1(index1), .index2(index2), 
    .index3(index3), .index4(index4));
    
    //MEM/WB Pipeline Register Module
    MEM_WB MEMWB(.clk(clk), .reset(reset), .MemtoReg(EX_MEM_MemtoReg), .RegWrite(EX_MEM_RegWrite), .Read_Data(Read_Data), 
    .Result_EX_MEM(EX_MEM_Result), .Rd_EX_MEM(EX_MEM_RD), .MemtoReg_MEM_WB(MEM_WB_MemtoReg), .RegWrite_MEM_WB(MEM_WB_RegWrite), 
    .Read_Data_MEM_WB(MEM_WB_Read_Data), .Result_MEM_WB(MEM_WB_Result), .Rd_MEM_WB(MEM_WB_RD));
    
    // Write Back (WB)
    Mux_2x1 muxlast(.A(MEM_WB_Result), .B(MEM_WB_Read_Data), .S(MEM_WB_MemtoReg), .Out(WriteData));
    
endmodule

//`timescale 1ns / 1ps

//module pipelinedProcessor_debug(
//    input clk,
//    input reset,
    
//    // Primary outputs
//    output wire [63:0] PC_In, PC_Out, ReadData1, ReadData2, WriteData, 
//                       Result, Read_Data, imm_data,
//    output wire [31:0] Instruction,
//    output wire [6:0] opcode,
//    output wire [4:0] rs1, rs2, rd,
//    output wire [1:0] ALUOp,
//    output wire [63:0] adder_out1, adder_out2,
//    output wire Branch, MemRead, MemWrite, MemtoReg, ALUSrc, RegWrite, Zero,
//    output wire [63:0] index0, index1, index2, index3, index4,
    
//    // DEBUG OUTPUTS - Pipeline Register Contents
//    output wire [31:0] IF_ID_Instruction_out,
//    output wire [63:0] IF_ID_PC_Out_out,
    
//    output wire ID_EX_Branch_out,
//    output wire ID_EX_MemRead_out,
//    output wire ID_EX_MemWrite_out,
//    output wire ID_EX_MemtoReg_out,
//    output wire ID_EX_ALUSrc_out,
//    output wire ID_EX_RegWrite_out,
//    output wire [1:0] ID_EX_ALUOp_out,
//    output wire [63:0] ID_EX_PC_Out_out,
//    output wire [63:0] ID_EX_ReadData1_out,
//    output wire [63:0] ID_EX_ReadData2_out,
//    output wire [63:0] ID_EX_Imm_Data_out,
//    output wire [4:0] ID_EX_RS1_out,
//    output wire [4:0] ID_EX_RS2_out,
//    output wire [4:0] ID_EX_RD_out,
//    output wire [3:0] ID_EX_Funct_out,
//    output wire [2:0] ID_EX_Funct3_out,
    
//    output wire EX_MEM_Branch_out,
//    output wire EX_MEM_Zero_out,
//    output wire EX_MEM_MemRead_out,
//    output wire EX_MEM_MemWrite_out,
//    output wire EX_MEM_MemtoReg_out,
//    output wire EX_MEM_RegWrite_out,
//    output wire [63:0] EX_MEM_Adder_Out_2_out,
//    output wire [63:0] EX_MEM_Result_out,
//    output wire [63:0] EX_MEM_Write_Data_out,
//    output wire [4:0] EX_MEM_RD_out,
    
//    output wire MEM_WB_MemtoReg_out,
//    output wire MEM_WB_RegWrite_out,
//    output wire [63:0] MEM_WB_Read_Data_out,
//    output wire [63:0] MEM_WB_Result_out,
//    output wire [4:0] MEM_WB_RD_out,
    
//    output wire [1:0] ForwardA_out,
//    output wire [1:0] ForwardB_out,
//    output wire sel_Branch_out
//);
    
//    // ========== INTERNAL WIRES (same as before) ==========
//    wire [63:0] muxmid_out;
//    wire [6:0] funct7;
//    wire [3:0] Funct, Operation;
//    wire [2:0] funct3;
//    wire sel_Branch;
    
//    // Pipeline register wires
//    wire [31:0] IF_ID_Instruction;
//    wire [63:0] IF_ID_PC_Out;
    
//    wire ID_EX_Branch, ID_EX_MemRead, ID_EX_MemWrite, ID_EX_MemtoReg, ID_EX_ALUSrc, ID_EX_RegWrite;
//    wire [1:0] ID_EX_ALUOp;
//    wire [63:0] ID_EX_PC_Out, ID_EX_ReadData1, ID_EX_ReadData2, ID_EX_Imm_Data;
//    wire [4:0] ID_EX_RS1, ID_EX_RS2, ID_EX_RD;
//    wire [3:0] ID_EX_Funct;
//    wire [2:0] ID_EX_Funct3;
    
//    wire EX_MEM_Branch, EX_MEM_Zero, EX_MEM_MemRead, EX_MEM_MemWrite, EX_MEM_MemtoReg, EX_MEM_RegWrite;
//    wire [63:0] EX_MEM_Adder_Out_2, EX_MEM_Result, EX_MEM_Write_Data;
//    wire [4:0] EX_MEM_RD;
    
//    wire MEM_WB_MemtoReg, MEM_WB_RegWrite;
//    wire [63:0] MEM_WB_Read_Data, MEM_WB_Result;
//    wire [4:0] MEM_WB_RD;
    
//    wire [1:0] ForwardA, ForwardB;
//    wire [63:0] mux_ReadData1, mux_ReadData2;
    
//    // ========== CONNECT DEBUG OUTPUTS ==========
//    assign IF_ID_Instruction_out = IF_ID_Instruction;
//    assign IF_ID_PC_Out_out = IF_ID_PC_Out;
    
//    assign ID_EX_Branch_out = ID_EX_Branch;
//    assign ID_EX_MemRead_out = ID_EX_MemRead;
//    assign ID_EX_MemWrite_out = ID_EX_MemWrite;
//    assign ID_EX_MemtoReg_out = ID_EX_MemtoReg;
//    assign ID_EX_ALUSrc_out = ID_EX_ALUSrc;
//    assign ID_EX_RegWrite_out = ID_EX_RegWrite;
//    assign ID_EX_ALUOp_out = ID_EX_ALUOp;
//    assign ID_EX_PC_Out_out = ID_EX_PC_Out;
//    assign ID_EX_ReadData1_out = ID_EX_ReadData1;
//    assign ID_EX_ReadData2_out = ID_EX_ReadData2;
//    assign ID_EX_Imm_Data_out = ID_EX_Imm_Data;
//    assign ID_EX_RS1_out = ID_EX_RS1;
//    assign ID_EX_RS2_out = ID_EX_RS2;
//    assign ID_EX_RD_out = ID_EX_RD;
//    assign ID_EX_Funct_out = ID_EX_Funct;
//    assign ID_EX_Funct3_out = ID_EX_Funct3;
    
//    assign EX_MEM_Branch_out = EX_MEM_Branch;
//    assign EX_MEM_Zero_out = EX_MEM_Zero;
//    assign EX_MEM_MemRead_out = EX_MEM_MemRead;
//    assign EX_MEM_MemWrite_out = EX_MEM_MemWrite;
//    assign EX_MEM_MemtoReg_out = EX_MEM_MemtoReg;
//    assign EX_MEM_RegWrite_out = EX_MEM_RegWrite;
//    assign EX_MEM_Adder_Out_2_out = EX_MEM_Adder_Out_2;
//    assign EX_MEM_Result_out = EX_MEM_Result;
//    assign EX_MEM_Write_Data_out = EX_MEM_Write_Data;
//    assign EX_MEM_RD_out = EX_MEM_RD;
    
//    assign MEM_WB_MemtoReg_out = MEM_WB_MemtoReg;
//    assign MEM_WB_RegWrite_out = MEM_WB_RegWrite;
//    assign MEM_WB_Read_Data_out = MEM_WB_Read_Data;
//    assign MEM_WB_Result_out = MEM_WB_Result;
//    assign MEM_WB_RD_out = MEM_WB_RD;
    
//    assign ForwardA_out = ForwardA;
//    assign ForwardB_out = ForwardB;
//    assign sel_Branch_out = sel_Branch;
    
//    // ========== Instruction Fetch (IF) Stage ==========
//    Adder A1(.A(PC_Out), .B(64'd4), .Out(adder_out1));
//    Mux_2x1 muxfirst(.A(adder_out1), .B(adder_out2), .S(sel_Branch && ID_EX_Branch), .Out(PC_In));
//    Program_Counter PC(.clk(clk), .reset(reset), .PC_In(PC_In), .PC_Out(PC_Out));
//    Instruction_Memory IM(.Inst_Address(PC_Out), .Instruction(Instruction));
    
//    // IF/ID Pipeline Register
//    IF_ID IFID(
//        .clk(clk), 
//        .reset(reset), 
//        .PC_Out(PC_Out), 
//        .Instruction(Instruction), 
//        .PC_Out_IF_ID(IF_ID_PC_Out),
//        .Instruction_IF_ID(IF_ID_Instruction)
//    );
    
//    // ========== Instruction Decode (ID) Stage ==========
//    Instruction_Parser IP(
//        .Instruction(IF_ID_Instruction), 
//        .Opcode(opcode), 
//        .RD(rd), 
//        .Funct3(funct3), 
//        .RS1(rs1), 
//        .RS2(rs2), 
//        .Funct7(funct7)
//    );
    
//    Imm_Gen Immgen(.Instruction(IF_ID_Instruction), .Imm(imm_data));
    
//    Control_Unit cu(
//        .Opcode(opcode), 
//        .Branch(Branch), 
//        .MemRead(MemRead), 
//        .MemtoReg(MemtoReg), 
//        .ALUOp(ALUOp), 
//        .MemWrite(MemWrite), 
//        .ALUSrc(ALUSrc), 
//        .RegWrite(RegWrite)
//    );
    
//    RegisterFile rf(
//        .clk(clk), 
//        .reset(reset), 
//        .WriteData(WriteData), 
//        .RS1(rs1), 
//        .RS2(rs2), 
//        .RD(MEM_WB_RD), 
//        .RegWrite(MEM_WB_RegWrite), 
//        .ReadData1(ReadData1), 
//        .ReadData2(ReadData2)
//    );
    
//    assign Funct = {IF_ID_Instruction[30], IF_ID_Instruction[14:12]};  // FIXED: Use IF_ID_Instruction
    
//    // ID/EX Pipeline Register
//    ID_EX IDEX(
//        .clk(clk), 
//        .reset(reset), 
//        .Branch(Branch), 
//        .MemRead(MemRead), 
//        .MemWrite(MemWrite), 
//        .MemtoReg(MemtoReg), 
//        .ALUSrc(ALUSrc), 
//        .RegWrite(RegWrite), 
//        .ALUOp(ALUOp), 
//        .PC_Out(IF_ID_PC_Out), 
//        .ReadData1(ReadData1), 
//        .ReadData2(ReadData2), 
//        .Imm_Data(imm_data), 
//        .RS1(rs1), 
//        .RS2(rs2), 
//        .RD(rd), 
//        .Funct(Funct), 
//        .Funct3(funct3), 
//        .ID_EX_Branch(ID_EX_Branch), 
//        .ID_EX_MemRead(ID_EX_MemRead), 
//        .ID_EX_MemWrite(ID_EX_MemWrite), 
//        .ID_EX_MemtoReg(ID_EX_MemtoReg), 
//        .ID_EX_ALUSrc(ID_EX_ALUSrc), 
//        .ID_EX_RegWrite(ID_EX_RegWrite), 
//        .ID_EX_ALUOp(ID_EX_ALUOp), 
//        .ID_EX_PC_Out(ID_EX_PC_Out), 
//        .ID_EX_ReadData1(ID_EX_ReadData1), 
//        .ID_EX_ReadData2(ID_EX_ReadData2), 
//        .ID_EX_Imm_Data(ID_EX_Imm_Data),
//        .ID_EX_RS1(ID_EX_RS1), 
//        .ID_EX_RS2(ID_EX_RS2), 
//        .ID_EX_RD(ID_EX_RD),
//        .ID_EX_Funct(ID_EX_Funct), 
//        .ID_EX_Funct3(ID_EX_Funct3)
//    );
    
//    // ========== Execute (EX) Stage ==========
//    Adder A2(.A(ID_EX_PC_Out), .B(ID_EX_Imm_Data << 1), .Out(adder_out2));  // imm_data * 2
    
//    Forwarding_Unit FU(
//        .ID_EX_RS1(ID_EX_RS1), 
//        .ID_EX_RS2(ID_EX_RS2), 
//        .EX_MEM_RD(EX_MEM_RD), 
//        .MEM_WB_RD(MEM_WB_RD), 
//        .EX_MEM_RegWrite(EX_MEM_RegWrite), 
//        .MEM_WB_RegWrite(MEM_WB_RegWrite),
//        .ForwardA(ForwardA), 
//        .ForwardB(ForwardB)
//    );
    
//    Mux_3x1 Mux_3x1_A(
//        .A(ID_EX_ReadData1), 
//        .B(WriteData), 
//        .C(EX_MEM_Result), 
//        .sel(ForwardA), 
//        .O(mux_ReadData1)
//    );
    
//    Mux_3x1 Mux_3x1_B(
//        .A(ID_EX_ReadData2), 
//        .B(WriteData), 
//        .C(EX_MEM_Result), 
//        .sel(ForwardB), 
//        .O(mux_ReadData2)
//    );
    
//    Mux_2x1 muxmid(
//        .A(mux_ReadData2), 
//        .B(ID_EX_Imm_Data), 
//        .S(ID_EX_ALUSrc), 
//        .Out(muxmid_out)
//    );
    
//    ALU_Control aluc(
//        .ALUOp(ID_EX_ALUOp), 
//        .Funct(ID_EX_Funct), 
//        .Operation(Operation)
//    );
    
//    Branch_unit BU(
//        .Funct3(ID_EX_Funct3), 
//        .ReadData1(mux_ReadData1), 
//        .ReadData2(mux_ReadData2), 
//        .addermuxselect(sel_Branch)
//    );
    
//    ALU64bit ALU(
//        .A(mux_ReadData1), 
//        .B(muxmid_out), 
//        .ALUOp(Operation), 
//        .Zero(Zero), 
//        .Result(Result)
//    );
    
//    // EX/MEM Pipeline Register
//    EX_MEM EXMEM(
//        .clk(clk), 
//        .reset(reset), 
//        .Branch(ID_EX_Branch), 
//        .Zero(sel_Branch), 
//        .MemRead(ID_EX_MemRead), 
//        .MemWrite(ID_EX_MemWrite), 
//        .MemtoReg(ID_EX_MemtoReg), 
//        .RegWrite(ID_EX_RegWrite), 
//        .Adder_Out_2(adder_out2), 
//        .Result(Result), 
//        .Write_Data(mux_ReadData2), 
//        .RD(ID_EX_RD),
//        .EX_MEM_Branch(EX_MEM_Branch), 
//        .EX_MEM_Zero(EX_MEM_Zero), 
//        .EX_MEM_MemRead(EX_MEM_MemRead), 
//        .EX_MEM_MemWrite(EX_MEM_MemWrite), 
//        .EX_MEM_MemtoReg(EX_MEM_MemtoReg), 
//        .EX_MEM_RegWrite(EX_MEM_RegWrite), 
//        .EX_MEM_Adder_Out_2(EX_MEM_Adder_Out_2), 
//        .EX_MEM_Result(EX_MEM_Result), 
//        .EX_MEM_Write_Data(EX_MEM_Write_Data), 
//        .EX_MEM_RD(EX_MEM_RD)
//    );
    
//    // ========== Memory (MEM) Stage ==========
//    Data_Memory DM(
//        .clk(clk), 
//        .MemWrite(EX_MEM_MemWrite), 
//        .MemRead(EX_MEM_MemRead), 
//        .Mem_Addr(EX_MEM_Result), 
//        .Write_Data(EX_MEM_Write_Data), 
//        .Read_Data(Read_Data), 
//        .index0(index0), 
//        .index1(index1), 
//        .index2(index2), 
//        .index3(index3), 
//        .index4(index4)
//    );
    
//    // MEM/WB Pipeline Register
//    MEM_WB MEMWB(
//        .clk(clk), 
//        .reset(reset), 
//        .MemtoReg(EX_MEM_MemtoReg), 
//        .RegWrite(EX_MEM_RegWrite), 
//        .Read_Data(Read_Data), 
//        .Result_EX_MEM(EX_MEM_Result), 
//        .Rd_EX_MEM(EX_MEM_RD), 
//        .MemtoReg_MEM_WB(MEM_WB_MemtoReg), 
//        .RegWrite_MEM_WB(MEM_WB_RegWrite), 
//        .Read_Data_MEM_WB(MEM_WB_Read_Data), 
//        .Result_MEM_WB(MEM_WB_Result), 
//        .Rd_MEM_WB(MEM_WB_RD)
//    );
    
//    // ========== Write Back (WB) Stage ==========
//    Mux_2x1 muxlast(
//        .A(MEM_WB_Result), 
//        .B(MEM_WB_Read_Data), 
//        .S(MEM_WB_MemtoReg), 
//        .Out(WriteData)
//    );
    
//endmodule