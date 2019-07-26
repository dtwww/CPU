`timescale 1ns / 1ps
module testbench();
reg clk;
reg reset;
reg [7:0]switches;//²¦¼ü
reg btn_c, btn_u, btn_d;
wire [15:0]leds;
    
    top t1 (clk,reset,switches,btn_c, btn_u, btn_d,leds);
    always
         begin
             clk <= 1; # 5; clk <= 0; # 5;
         end
    initial
        begin
            switches <= 8'b00010010;
            btn_c <= 1;
            btn_u <= 0;
            btn_d <= 0;
            reset <= 1; # 22; reset <= 0;                        
            $display("Running testbench"); 
        end
endmodule
