`timescale 1ns / 1ps

module hazardProcessor(
    input clk,
    input reset,
    output reg [63:0] PC_In, PC_Out, ReadData1, ReadData2, WriteData, Result, Read_Data, imm_data,
    output reg [31:0] Instruction,
    output reg [6:0] opcode,
    output reg [4:0] rs1, rs2, rd,
    output reg [1:0] ALUOp,
    output reg [63:0] adder_out1, adder_out2,
    output reg Branch, MemRead, MemWrite, MemtoReg, ALUSrc, RegWrite, Zero, stall,
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
    wire Branch, MemRead, MemWrite, MemtoReg, ALUSrc, RegWrite, Zero, sel_Branch, stall;
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
    
    // extra wires
    wire [63:0] mux_ReadData1, mux_ReadData2;
    
    // Instruction Fetch (IF) Modules
    
    Adder A1(.A(PC_Out), .B(64'd4), .Out(adder_out1));
//    Mux_2x1 muxfirst(.A(adder_out1), .B(EX_MEM_Adder_Out_2), .S(EX_MEM_Zero && EX_MEM_Branch), .Out(PC_In));
    Mux_2x1 muxfirst(.A(adder_out1), .B(adder_out2), .S(sel_Branch && Branch), .Out(PC_In));
    Program_Counter PC(.clk(clk), .reset(reset), .stall(stall), .PC_In(PC_In), .PC_Out(PC_Out));
    Instruction_Memory IM(.Inst_Address(PC_Out), .Instruction(Instruction));
    
    //IF/ID Pipeline Register Module
    IF_ID IFID(
    .clk(clk), .reset(reset), .stall(stall), .PC_Out(PC_Out), .Instruction(Instruction), .PC_Out_IF_ID(IF_ID_PC_Out),
    .Instruction_IF_ID(IF_ID_Instruction));
    
    // Instruction Decode (ID) Modules / Register File Read
    Instruction_Parser IP(.Instruction(IF_ID_Instruction), .Opcode(opcode), .RD(rd), .Funct3(funct3), 
    .RS1(rs1), .RS2(rs2), .Funct7(funct7));
    Control_Unit cu(.stall(stall), .Opcode(opcode), .Branch(Branch), .MemRead(MemRead), .MemtoReg(MemtoReg), .ALUOp(ALUOp), 
    .MemWrite(MemWrite), .ALUSrc(ALUSrc), .RegWrite(RegWrite));
    Hazard_Detection HD( .RS1(rs1), .RS2(rs2), .Rd_ID_EX(ID_EX_RD), .MemRead_ID_EX(ID_EX_MemRead), .stall(stall));
//    Imm_Gen Immgen(.Instruction(Instruction), .Imm(imm_data));
    Imm_Gen Immgen(.Instruction(IF_ID_Instruction), .Imm(imm_data));
    Adder A2(.A(IF_ID_PC_Out), .B(imm_data<<1), .Out(adder_out2));
    RegisterFile rf(.clk(clk), .reset(reset), .WriteData(WriteData), .RS1(rs1), .RS2(rs2), .RD(MEM_WB_RD), 
    .RegWrite(MEM_WB_RegWrite), .ReadData1(ReadData1), .ReadData2(ReadData2));
    Branch_unit BU(.Funct3(funct3), .ReadData1(ReadData1), .ReadData2(ReadData2), .addermuxselect(sel_Branch));
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
    Forwarding_Unit FU(ID_EX_RS1, ID_EX_RS2, EX_MEM_RD, MEM_WB_RD, EX_MEM_RegWrite, MEM_WB_RegWrite,
    ForwardA, ForwardB);
    Mux_3x1 Mux_3x1_A(.A(ID_EX_ReadData1), .B(WriteData), .C(EX_MEM_Result), .sel(ForwardA), .O(mux_ReadData1));
    Mux_3x1 Mux_3x1_B(.A(ID_EX_ReadData2), .B(WriteData), .C(EX_MEM_Result), .sel(ForwardB), .O(mux_ReadData2));
    Mux_2x1 muxmid(.A(ID_EX_ReadData2), .B(ID_EX_Imm_Data), .S(ID_EX_ALUSrc), .Out(muxmid_out));
//    Mux_2x1 muxmid(.A(mux_ReadData2), .B(ID_EX_Imm_Data), .S(ALUSrc), .Out(muxmid_out));
    ALU_Control aluc(.ALUOp(ID_EX_ALUOp), .Funct(ID_EX_Funct), .Operation(Operation));
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