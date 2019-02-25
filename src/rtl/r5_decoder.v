//======================================================================
//
// r5_decoder.v
// ------------
// Instruction decoder for the RISC-V processor.
// An instruction can be loaded and its fields are decoded.
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

module r5_decoder (
                   input wire           clk,
                   input wire           reset_n,

                   input wire           instruction,
                   input wire           decode,

                   output wire [2 : 0]  instr_type,
                   output wire [4 : 0]  rs1,
                   output wire [4 : 0]  rs2,
                   output wire [4 : 0]  rd
                  );


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg [31 : 0] instruction_reg;
  reg [31 : 0] instruction_we;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg [31 : 0] tmp_src0_data;
  reg [31 : 0] tmp_src1_data;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  // We do basic decoding here.
  //----------------------------------------------------------------
  assign rs1 = instruction_reg[19 : 15];
  assign rs2 = instruction_reg[24 : 20];
  assign rd  = instruction_reg[11 :  7];


  //----------------------------------------------------------------
  // reg_update
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with asynchronous
  // active low reset.
  //----------------------------------------------------------------
  always @ (posedge clk or negedge reset_n)
    begin : reg_update
      integer i;

      if (!reset_n)
        begin
          instruction_reg <= 32'h0;

        end
      else
        begin
          if (decode)
            instruction_reg <= instruction;
        end
    end // reg_update

endmodule // r5_decoder

//======================================================================
// EOF r5_decoder.v
//======================================================================
