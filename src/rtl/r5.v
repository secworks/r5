//======================================================================
//
// r5.v
// ----
// Top level of the RISC-V processor.
//
//
// Author: Joachim Strombergson
// Copyright (c) 2019, Assured AB
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or
// without modification, are permitted provided that the following
// conditions are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

module r5(
          input wire           clk,
          input wire           reset_n,

          output wire          cs,
          output wire          we,

          output wire [31 : 0] address,
          output wire [31 : 0] write_data,
          input wire [31 : 0]  read_data,
          input wire           ready
         );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter BOOT_VECTOR = 32'h00001000;

  localparam CTRL_RESET       = 5'h00;
  localparam CTRL_BOOT        = 5'h01;
  localparam CTRL_INS_FETCH0  = 5'h04;
  localparam CTRL_INS_FETCH1  = 5'h05;
  localparam CTRL_INS_DECODE0 = 5'h08;
  localparam CTRL_INS_DECODE1 = 5'h09;
  localparam CTRL_OP_READ     = 5'h10;
  localparam CTRL_EXE         = 5'h12;
  localparam CTRL_WB          = 5'h14;
  localparam CTRL_JMP         = 5'h16;
  localparam CTRL_RD_DATA0    = 5'h18;
  localparam CTRL_RD_DATA1    = 5'h19;
  localparam CTRL_WR_DATA0    = 5'h1c;
  localparam CTRL_WR_DATA1    = 5'h1d;
  localparam CTRL_ERROR       = 5'h1f;


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg cs_reg;
  reg cs_new;
  reg cs_we;

  reg we_reg;
  reg we_new;
  reg we_we;

  reg ready_reg;

  reg [31 : 0] address_reg;
  reg [31 : 0] address_new;
  reg          address_we;

  reg [31 : 0] write_data_reg;
  reg [31 : 0] write_data_new;
  reg          write_data_we;

  reg [31 : 0] read_data_reg;
  reg          read_data_we;

  reg [31 : 0] pc_reg;
  reg [31 : 0] pc_new;
  reg [31 : 0] pc_jmp_addr;
  reg          pc_we;
  reg          pc_inc;
  reg          pc_rst;
  reg          pc_jmp;

  reg [31 : 0] instr_reg;
  reg          instr_we;


  reg [4 : 0] r5_ctrl_reg;
  reg [4 : 0] r5_ctrl_new;
  reg         r5_ctrl_we;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  wire [31 : 0] rs1_data;
  wire [31 : 0] rs2_data;
  reg [31 : 0]  rd_data;
  reg           rd_we;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign cs         = cs_reg;
  assign we         = we_reg;
  assign address    = address_reg;
  assign write_data = write_data_reg;


  //----------------------------------------------------------------
  // Instantiations.
  //----------------------------------------------------------------
  r5_regfile regfile(
                     .clk(clk),
                     .reset_n(reset_n),
                     .rs1(instr_reg[19 : 15]),
                     .rs1_data(rs1_data),
                     .rs2(instr_reg[24 : 20]),
                     .rs2_data(rs2_data),
                     .rd(instr_reg[11 : 7]),
                     .rd_we(rd_we),
                     .rd_data(rd_data)
                    );


  //----------------------------------------------------------------
  // reg_update
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with asynchronous
  // active low reset.
  //----------------------------------------------------------------
  always @ (posedge clk or negedge reset_n)
    begin : reg_update
      if (!reset_n)
        begin
          pc_reg         <= 32'h0;
          cs_reg         <= 1'h0;
          we_reg         <= 1'h0;
          ready_reg      <= 1'h0;
          instr_reg      <= 32'h0;
          address_reg    <= 32'h0;
          read_data_reg  <= 32'h0;
          write_data_reg <= 32'h0;
          r5_ctrl_reg    <= CTRL_RESET;
        end
      else
        begin
          ready_reg <= ready;

          if (cs_we)
            cs_reg <= cs_new;

          if (we_we)
            we_reg <= we_new;

          if (instr_we)
            instr_reg <= read_data;

          if (address_we)
            address_reg <= address_new;

          if (write_data_we)
            write_data_reg <= write_data_new;

          if (read_data_we)
            read_data_reg <= read_data;

          if (pc_we)
            pc_reg <= pc_new;

          if (r5_ctrl_we)
            r5_ctrl_reg <= r5_ctrl_new;

        end
    end // reg_update


  //----------------------------------------------------------------
  // pc_logic
  // The logic implementing the program counter (PC) updates
  //----------------------------------------------------------------
  always @*
    begin : pc_logic
      pc_new = 32'h0;
      pc_we  = 1'h0;

      if (pc_rst)
        begin
          pc_new = BOOT_VECTOR;
          pc_we  = 1'h1;
        end

      if (pc_inc)
        begin
          pc_new = pc_reg + 3'h4;
          pc_we  = 1'h1;
        end

      if (pc_jmp)
        begin
          pc_new = pc_jmp_addr;
          pc_we  = 1'h1;
        end
    end


  //----------------------------------------------------------------
  // decode_logic
  //----------------------------------------------------------------
  always @*
    begin : decode_logic
      reg [6 : 0] opcode;
      reg [4 : 0] rs1;
      reg [4 : 0] rs2;
      reg [4 : 0] rd;
      reg [2 : 0] funct3;
      reg [7 : 0] funct7;
      reg [11 : 0] immi;
      reg [11 : 0] imms;
      reg [12 : 0] immsb;
      reg [19 : 0] immu;
      reg [19 : 0] immuj;

      // Extract all possible fields from the instruction.
      opcode = instr_reg[06 : 00];
      rs1    = instr_reg[19 : 15];
      rs2    = instr_reg[24 : 20];
      rd     = instr_reg[11 : 07];
      funct3 = instr_reg[14 : 12];
      funct7 = instr_reg[31 : 25];
      immi   = instr_reg[31 : 20];
      imms   = {instr_reg[31 : 25], instr_reg[11 : 07]};
    end


  //----------------------------------------------------------------
  // r5_ctrl
  // Main control FSM.
  //----------------------------------------------------------------
  always @*
    begin : r5_ctrl
      cs_new         = 1'h0;
      cs_we          = 1'h0;
      we_new         = 1'h0;
      we_we          = 1'h0;
      address_new    = 32'h0;
      address_we     = 1'h0;
      write_data_new = 32'h0;
      write_data_we  = 1'h0;
      instr_we       = 1'h0;
      read_data_we   = 1'h0;
      pc_jmp_addr    = 32'h0;
      pc_rst         = 1'h0;
      pc_inc         = 1'h0;
      pc_jmp         = 1'h0;
      rd_we          = 1'h0;
      rd_data        = 32'h0;
      r5_ctrl_new    = CTRL_RESET;
      r5_ctrl_we     = 1'h0;

      case (r5_ctrl_reg)
        CTRL_RESET:
          begin
            r5_ctrl_new    = CTRL_BOOT;
            r5_ctrl_we     = 1'h1;
          end

        CTRL_BOOT:
          begin
            pc_rst      = 1'h1;
            r5_ctrl_new = CTRL_INS_FETCH0;
            r5_ctrl_we  = 1'h1;
          end

        CTRL_INS_FETCH0:
          begin
            cs_new      = 1'h1;
            cs_we       = 1'h1;
            address_new = pc_reg;
            address_we  = 1'h1;
            r5_ctrl_new = CTRL_INS_FETCH1;
            r5_ctrl_we  = 1'h1;
          end

        CTRL_INS_FETCH1:
          begin
            if (ready_reg)
              begin
                cs_new      = 1'h0;
                cs_we       = 1'h1;
                instr_we    = 1'h1;
                pc_inc      = 1'h1;
                r5_ctrl_new = CTRL_INS_DECODE0;
                r5_ctrl_we  = 1'h1;
              end
          end

        CTRL_INS_DECODE0:
          begin
          end

        CTRL_INS_DECODE1:
          begin
          end

        CTRL_OP_READ:
          begin
          end

        CTRL_EXE:
          begin
          end

        CTRL_WB:
          begin
          end

        CTRL_JMP:
          begin
          end

        CTRL_RD_DATA0:
          begin
          end

        CTRL_RD_DATA1:
          begin
          end

        CTRL_WR_DATA0:
          begin
          end

        CTRL_WR_DATA1:
          begin
          end

        CTRL_ERROR:
          begin
          end

    default:
    begin
    end

      endcase // case (r5_ctrl_reg)
    end // r5_ctrl

endmodule // r5

//======================================================================
// EOF r5.v
//======================================================================
