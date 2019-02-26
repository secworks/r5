//======================================================================
//
// r5_regfile.v
// ------------
// Register file implementation for the RISC-V processor.
// Reg reads are combinational.
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

module r5_regfile (
                   input wire           clk,
                   input wire           reset_n,

                   input wire [4 : 0]   rs1,
                   output wire [31 : 0] rs1_data,

                   input wire [4 : 0]   rs2,
                   output wire [31 : 0] rs2_data,

                   input wire [4 : 0]   rd,
                   input wire           rd_we,
                   input wire [31 : 0]  rd_data
                  );


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg [31 : 0] regs [1 : 31];


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg [31 : 0] tmp_rs1_data;
  reg [31 : 0] tmp_rs2_data;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign rs1_data = tmp_rs1_data;
  assign rs2_data = tmp_rs2_data;


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
          for (i = 1 ; i < 32 ; i = i + 1)
            regs[i] <= 32'h0;
        end
      else
        begin
          if (rd_we && (rd > 0))
            regs[rd] <= rd_data;
        end
    end // reg_update


  //----------------------------------------------------------------
  //----------------------------------------------------------------
  always @*
    begin : rs1_mux
      if (rs1 > 0)
        tmp_rs1_data = regs[rs1];
      else
        tmp_rs1_data = 32'h0;
    end


  //----------------------------------------------------------------
  //----------------------------------------------------------------
  always @*
    begin :rs2_mux
      if (rs2 > 0)
        tmp_rs2_data = regs[rs2];
      else
        tmp_rs2_data = 32'h0;
    end

endmodule // r5_regfile

//======================================================================
// EOF r5_regfile.v
//======================================================================
