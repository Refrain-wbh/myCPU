module regfile(
    input         clk,
    // READ PORT 1
    input  [ 4:0] raddr1,
    output [31:0] rdata1,
    // READ PORT 2
    input  [ 4:0] raddr2,
    output [31:0] rdata2,
    // WRITE PORT
    input         we,       //write enable, HIGH valid
    input  [ 4:0] waddr,
    input  [31:0] wdata
);
reg [31:0] rf[31:0];

//WRITE
always @(posedge clk) begin
    if (we) rf[waddr]<= wdata;
end

//READ OUT 1
assign rdata1 = (raddr1==5'b0) ? 32'b0 : rf[raddr1];

//READ OUT 2
assign rdata2 = (raddr2==5'b0) ? 32'b0 : rf[raddr2];

endmodule


module HILO(
    input clk,
    input wen_HI,
    input [31:0] wHI,
    input wen_LO,
    input [31:0] wLO,
    output [31:0] rHI,
    output [31:0] rLO
);
    reg [31:0] reg_HI;
    reg [31:0] reg_LO;
    
    always @(posedge clk) begin
        if(wen_HI)
            reg_HI<=wHI;
    end
    always @(posedge clk) begin
        if(wen_LO)
            reg_LO<=wLO;
    end
    assign rHI=reg_HI;
    assign rLO=reg_LO;
endmodule