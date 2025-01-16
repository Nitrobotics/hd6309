`timescale 1ns / 1ps

module hd6309e(
    input   [7:0] D,
    output  [7:0] DOut,
    output  [15:0] ADDR,
    output  RnW,
    input   E,
    input   Q,
    output  BS,
    output  BA,
    input   nIRQ,
    input   nFIRQ,
    input   nNMI,
    output  AVMA,
    output  BUSY,
    output  LIC,
    input 	nHALT,	 
    input   nRESET

    );



hd6309i cpucore (.D(D), .DOut(DOut), .ADDR(ADDR), .RnW(RnW), .E(E), .Q(Q), .BS(BS), .BA(BA), .nIRQ(nIRQ), .nFIRQ(nFIRQ), 
                .nNMI(nNMI), .AVMA(AVMA), .BUSY(BUSY), .LIC(LIC), .nHALT(nHALT), .nRESET(nRESET), .nDMABREQ(1'b1)
                );


endmodule
