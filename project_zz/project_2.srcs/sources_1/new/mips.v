module top #(parameter WIDTH =32 ,ADDR_WIDTH=16)(clk,reset,switches,btn_c, btn_u, btn_d, leds);
    input         clk;
    input         reset;
    input [7:0]   switches;//拨键
    input         btn_c, btn_u, btn_d;//按键
    output [15:0] leds;
    wire             memread,memwrite;
    wire [ADDR_WIDTH-1:0] adr;
    wire [WIDTH-1:0] writedata;
    wire [WIDTH-1:0] memdata;
    wire [15:0]      leds;
    wire [7:0]       switches;
    wire btn_c, btn_u, btn_d;
    wire btn_c_o, btn_u_o, btn_d_o;
    reg count=0;
    reg clk1=0;

    initial
        begin
	       clk1<=0;
        end

    always @ (posedge clk)
    begin 
        if(count == 1)
            begin
                clk1 <= ~clk1;
                count <= 0;
            end
        else
          count <= count+1;
    end

    mips     #(WIDTH) dut(clk1, reset, memdata, memread, memwrite, adr, writedata);
    exmemory #(WIDTH) exmem(clk1, reset, memwrite, memread, adr, writedata, memdata, btn_c_o, btn_u_o, btn_d_o, switches, leds);
    key_fangdou fd1(clk1,btn_c,btn_c_o); 
    key_fangdou fd2(clk1,btn_u,btn_u_o);
    key_fangdou fd3(clk1,btn_d,btn_d_o);

endmodule

module exmemory #(parameter WIDTH =32, ADDR_WIDTH=16)(
    input             clk,
    input             reset,
    input             memwrite,memread,
    input [ADDR_WIDTH-1:0] adr,
    input [WIDTH-1:0] writedata,
    output reg [WIDTH-1:0] memdata,
    input btn_c,btn_u,btn_d,
    input [7: 0] switches,
    output reg [15:0] leds);

    wire [31 : 0] romData;
    wire [31 : 0] ramData;
    wire RamWrite;
    wire [4:0] IOState;

    button btn(clk, reset, btn_c, btn_u, btn_d, IOState);

    assign RamWrite = ~reset & memwrite & (adr[15:12] == 4'h1);

    rom ROM(adr[11:2], romData);
    ram RAM(clk, RamWrite, adr[11:2], writedata, ramData);

    initial begin
        leds = 0;
    end

    always @ ( * )
    begin
        case (adr[15:12])
            4'h0: begin
                      if (memread)
                          memdata <= romData;
                  end
            4'h1: begin
                      if (memread)
                          memdata <= ramData;
                  end
            4'hf: begin
                case (adr)
			    16'hfffc: memdata <= {24'b0, switches[7:0]};
                16'hfff4: memdata <= {27'b0, IOState[4:0]};
                endcase
            end
        endcase
    end

    always @ (posedge clk) 
    begin
        if (reset) 
            begin
                leds <= 0;
            end
        else if (memwrite && ~RamWrite) 
            begin
                case (adr)
                    16'hfff0: leds <= {writedata[15:0]};
                endcase
            end
    end
endmodule

module button (
    input clk, reset,
    input btn_c,btn_u,btn_d,
    output reg [4:0] IOState
    );

    initial begin
        IOState = 0;
    end

    always @ (posedge clk) begin
        if (reset) IOState <= 5'b00000;
        else if (btn_c) IOState <= 5'b00001;//center
        else if (btn_u) IOState <= 5'b00010;//up
        else if (btn_d) IOState <= 5'b00100;//down
        //else if (btn_l) IOState <= 5'b01000;//left
        //else if (btn_r) IOState <= 5'b10000;//right
    end

endmodule // button

module ram #(parameter WIDTH = 32, ADDR_WIDTH = 10) (
    input                     clk,
    input                     write,
    input  [ADDR_WIDTH-1 : 0] addr,
    input  [WIDTH-1 : 0]      inData,
    output [WIDTH-1 : 0]      outData
    );

    reg [WIDTH-1 : 0] RAM[0:(1<<ADDR_WIDTH)-1];

    assign outData = RAM[addr];

    always @ (posedge clk) begin
        if (write)
            RAM[addr] <= inData;
    end

endmodule // ram

module rom #(parameter WIDTH = 32, ADDR_WIDTH = 10) (
    input  [ADDR_WIDTH-1 : 0] addr,
    output [WIDTH-1 : 0]      data
    );

    reg [WIDTH-1 : 0] ROM [0:(1<<ADDR_WIDTH)-1];

    initial begin
        $readmemh("F:/Mars/test.dat",ROM);
    end

    assign data = ROM[addr];

endmodule // rom

module mips #(parameter WIDTH=32,ADDR_WIDTH=16) 
          (input              clk,reset,
           input  [WIDTH-1:0] memdata,
           output             memread,memwrite,
           output [ADDR_WIDTH-1:0] adr,
           output [WIDTH-1:0] writedata);
    wire [31:0] instr;
    wire        memtoreg, irwrite, iord, pcen, regwrite, regdst, zero;
    wire [1:0]  alusrca, alusrcb, pcsource;
    wire [5:0]  aluop;
  
    controller cont(clk, reset, instr[31:26], instr[5:0],
                    zero, memread, memwrite, memtoreg, iord, irwrite,
                    pcen, regwrite, regdst, pcsource, alusrca, alusrcb, aluop);

    datapath #(WIDTH) dp(clk, reset, memdata, memtoreg, iord, pcen, regwrite, regdst,
                         irwrite, alusrca, alusrcb, pcsource, aluop, zero,
                         instr, adr, writedata);
  
endmodule

module controller(
        input             clk,reset,
        input       [5:0] op,
        input       [5:0] funct,
        input             zero,
        output reg        memread,memwrite,memtoreg,iord,irwrite,
        output            pcen,
        output reg       regwrite,regdst,
        output reg [1:0] pcsource,alusrca,alusrcb,
        output reg [5:0] aluop);
                reg [3:0] state, nextstate;
                reg       pcwrite, pcwritecond;
        
    parameter FETCH  = 4'b0000;//取指令
    parameter DECODE = 4'b0001;//分析指令
    parameter MTYPES = 4'b0010;
    parameter ITYPES = 4'b0011;
    //parameter JRUMPS = 4'b0100;
    parameter BEQS   = 4'b0101;
    parameter JUMPS  = 4'b1000;
    parameter ReadMS = 4'b1001;
    parameter WriteMS= 4'b1010;
    parameter IWriteToRegS = 4'b1011;
    parameter RITYPES = 4'b1100;
    parameter ROTYPES = 4'b1101;
    parameter MWriteToRegS = 4'b1110;
    parameter RWriteToRegS = 4'b1111;

    parameter RTYPE = 6'b000000;//R型指令
    parameter J     = 6'b000010;//j无条件转移
    parameter BEQ   = 6'b000100;//相等则跳转
    parameter ADDI  = 6'b001000;//立即数加
    parameter ANDI  = 6'b001100;//立即数与
    parameter ORI   = 6'b001101;//立即数或
    parameter LUI   = 6'b001111;//立即数装入寄存器高16位
    parameter LW    = 6'b100011;//取字
    parameter SW    = 6'b101011;//存字

    parameter SLL  = 6'b000000;//sll逻辑左移
    parameter SRL  = 6'b000010;//srl逻辑右移
    parameter ADD  = 6'b100000;//add加
    parameter ADDU = 6'b100001;//addu无符号加
    parameter SUB  = 6'b100010;//sub减
    parameter AND  = 6'b100100;//and与
    parameter OR   = 6'b100101;//or或
  
    always @(posedge clk)
        begin
            if(reset)
                state <= FETCH;
            else
                state <= nextstate;
        end

    always @(*)
        begin
            case(state)
                FETCH : nextstate <= DECODE;
                DECODE: case(op)
                            RTYPE:
                                case(funct)
                                    SLL:  nextstate <= RITYPES;
                                    SRL:  nextstate <= RITYPES;
                                    ADD:  nextstate <= ROTYPES;
                                    ADDU: nextstate <= ROTYPES;
                                    SUB:  nextstate <= ROTYPES;
                                    AND:  nextstate <= ROTYPES;
                                    OR:   nextstate <= ROTYPES;
                                endcase
                            J:      nextstate <= JUMPS;
                            BEQ:    nextstate <= BEQS;
                            ADDI:   nextstate <= ITYPES;
                            ANDI:   nextstate <= ITYPES;
                            ORI:    nextstate <= ITYPES;
                            LUI:    nextstate <= ITYPES;
                            LW:     nextstate <= MTYPES;
                            SW:     nextstate <= MTYPES;
                            default:nextstate <= FETCH;
                        endcase
                RITYPES: nextstate <= RWriteToRegS;
                ROTYPES: nextstate <= RWriteToRegS;
                ITYPES:  nextstate <= IWriteToRegS;
                BEQS:    nextstate <= FETCH;
                JUMPS:   nextstate <= FETCH;
                //JRUMPS:  nextstate <= FETCH;
                MTYPES: case(op)
                        LW:
                            nextstate <= ReadMS;
                        SW:
                            nextstate <= WriteMS;
                        endcase
                ReadMS:       nextstate <= MWriteToRegS;
                WriteMS:      nextstate <= FETCH;
                MWriteToRegS: nextstate <= FETCH;
                IWriteToRegS: nextstate <= FETCH;
                RWriteToRegS: nextstate <= FETCH;
                default:      nextstate <= FETCH;
            endcase
        end
    always @(*)
        begin
            irwrite  <= 0;
            pcwrite  <= 0;     pcwritecond <= 0;
            regwrite <= 0;     regdst      <= 0;
            memread  <= 0;     memwrite    <= 0;
            alusrca  <= 2'b00; alusrcb     <= 2'b00; aluop <= 6'b100000;
            pcsource <= 2'b00;
            iord     <= 0;     memtoreg    <= 0;
        case(state)
            FETCH:
                begin
                    irwrite<=1;//IR写
                    memread<=1;//存储器读指令
                    alusrcb<=2'b01;//PC+1
                    pcwrite<=1;//pcen置1
                    aluop<=6'b100000;//add
                end
            DECODE:
                begin
                    aluop<=6'b100000;//add
                    alusrcb<=2'b11;//跳转指令offset
                end
            MTYPES:
                begin
                    alusrca<=2'b01;
                    alusrcb<=2'b10;
                    aluop<=6'b100000;
                end
            ITYPES:
                begin
                    alusrca<=2'b01;
                    alusrcb<=2'b10;
                    case(op)
                        ADDI:
                            aluop<=6'b100000;
                        ANDI:
                            aluop<=6'b100100;
                        ORI:
                            aluop<=6'b110000;
                        LUI:
                            aluop<=6'b010001;
                    endcase
                end
/*            JRUMPS:
                begin
                    pcwrite<=1;
                    pcsource<=2'b11;
                end*/
            BEQS:
                begin
                    alusrca<=2'b01;
                    alusrcb<=2'b00;
                    aluop<=6'b100010;
                    pcsource<=2'b01;
                    pcwritecond<=1;
                end
            JUMPS:
                begin
                    pcwrite<=1;
                    pcsource<=2'b10;
                end
            ReadMS:
                begin
                    memread<=1;
                    iord<=1;
                end
            WriteMS:
                begin
                    memwrite<=1;
                    iord<=1;
                end
            IWriteToRegS:
                begin
                    memtoreg<=0;
                    regwrite<=1;
                    regdst<=0;
                end
            RITYPES:
                begin
                    alusrca<=2'b10;
                    alusrcb<=2'b00;
                    case(funct)
                        SLL: aluop <= 6'b000000;
                        SRL: aluop <= 6'b000010;
                    endcase
                end
            ROTYPES:
                begin
                    alusrca<=2'b01;
                    alusrcb<=2'b00;
                    case(funct)
                        ADD: aluop <= 6'b100000;
                        ADDU:aluop <= 6'b100001;
                        SUB: aluop <= 6'b100010;
                        AND: aluop <= 6'b100100;
                        OR:  aluop <= 6'b100101;
                    endcase
                end
            MWriteToRegS:
                begin
                    regdst<=0;
                    regwrite<=1;
                    memtoreg<=1;
                end
            RWriteToRegS:
                begin
                    regdst<=1;
                    regwrite<=1;
                    memtoreg<=0;
                end
        endcase
    end
    assign pcen = pcwrite | (pcwritecond & zero);
endmodule

//datapath
module datapath #(parameter WIDTH =32 ,ADDR_WIDTH=16)
        (input              clk, reset,
         input [WIDTH-1:0]  memdata,
         input              memtoreg, iord, pcen, regwrite, regdst, irwrite,
         input [1:0]        alusrca, alusrcb, pcsource,
         input [5:0]        aluop,
         output             zero,
         output [31:0]      instr,
         output [ADDR_WIDTH-1:0] adr,
         output [WIDTH-1:0] writedata);
         
    parameter CONST_ZERO = 32'b0;
    parameter CONST_ONE = 32'h4;
    wire [4:0] ra1, ra2, wa;
    wire [15:0] pc, nextpc;
    wire [WIDTH-1:0] md, rd1, rd2, wd, a, src1, src2, aluresult, aluout;
    wire [31:0] jp1;
    wire [31:0] ta1,ta2;
    wire [15:0] imme;
    wire [31:0] imme_extend;
    wire [31:0] imme_leftshift;
    wire [25:0] jump;
    wire [27:0] jump_after;
    
    //assign jp1 = {6'b000000,instr[25:0]};
    assign ta1 = {27'b0,instr[10:6]};
    assign ta2 = {16'b0,instr[15:0]};
    assign ra1 = instr[25:21];
    assign ra2 = instr[20:16];
    assign imme = instr[15:0];
    assign jump = instr[25:0];
    
    signextend signe(imme,imme_extend);//符号扩展指令的后16位
    leftshift2 ls(imme_extend,imme_leftshift);//符号扩展后再左移两位
    leftshift2 #(28) j_ls({2'b0,jump},jump_after);//J指令offset左移两位
    
    PC                pcreg(clk, reset, pcen, nextpc, pc);//程序计数器PC
    mux2    #(16) adrmux(pc, aluout[15:0], iord, adr);//PC右的二路选择器
    flopen  #(32) ir(clk, irwrite, memdata, instr);//指令寄存器
    flop    #(WIDTH) mdr(clk,memdata,md);//Memory data register
    mux2    #(WIDTH) wdmux(aluout, md, memtoreg, wd);//IR右下的二路选择器
    mux2             regmux(instr[20:16],instr[15:11],regdst,wa);//IR右的二路选择器
    regfile #(WIDTH) rf(clk, reset, regwrite, ra1, ra2, wa, wd, rd1, rd2);//寄存器堆
    flop    #(WIDTH) areg(clk, rd1, a);//锁存A
    flop    #(WIDTH) wrd(clk, rd2, writedata);//锁存B
    mux4    #(WIDTH) src1mux({16'b0,pc}, a, ta1, ta2, alusrca, src1);//A右边的四路选择器
    mux4    #(WIDTH) src2mux(writedata, CONST_ONE, imme_extend, imme_leftshift, alusrcb, src2);//B右边的四路选择器
    alu     #(WIDTH) alunit(src1, src2, aluop, aluresult);//Alu
    flop    #(WIDTH) res(clk, aluresult, aluout);//Aluout
    mux4    #(16) pcmux(aluresult[15:0], aluout[15:0], jump_after[15:0], 16'b0, pcsource, nextpc);//Aluout右的四路选择器
    zerodetect #(WIDTH) zd(aluresult, zero);//判断结果是否为0
endmodule

//算逻运算单元ALU
module alu #(parameter WIDTH=32)
      (input      [WIDTH-1:0] a,b,
       input      [5:0]       aluop,
       output reg [WIDTH-1:0] result);
    //wire [WIDTH-1:0] sum,slt,shamt;
    always @(*)
        begin
            case(aluop)
                6'b000000: result <= (b<<a);//sll
                6'b000010: result <= (b>>a);//srl
                6'b001000: result <= 32'b0;
                6'b100000: result <= (a+b);//add
                6'b100001: result <= (a+b);//add
                6'b100010: result <= (a-b);//sub
                6'b100100: result <= (a&b);//and
                6'b100101: result <= (a|b);//or
	            6'b010001: result <= (b<<16);//lui
	            6'b110000: result <= (32'h0000ffff & b) | a;
            endcase
        end
endmodule

//寄存器堆
module regfile #(parameter WIDTH = 32, ADDR_WIDTH = 5)
      (input clk,
      input reset,
      input regwrite,
      input  [ADDR_WIDTH-1:0] readAddr1,readAddr2,writeAddr,
      input  [WIDTH-1:0] writeData,
      output [WIDTH-1:0] readData1,readData2);
          reg [WIDTH-1:0] REG [(1<<ADDR_WIDTH)-1:0];
    initial
        begin
            REG[0] = 0;
            REG[1] = 1;
            REG[16] = 32'h000000a0;
        end
    always @(posedge clk)
        begin
            if(regwrite)
                REG[writeAddr] <= writeData;
        end
    assign readData1 = readAddr1 ? REG[readAddr1]:0;
    assign readData2 = readAddr2 ? REG[readAddr2]:0;
endmodule

//判断结果是否为0
module zerodetect #(parameter WIDTH = 32)
    (input [WIDTH-1:0] a,
    output             y);
    assign y = (a==0);
endmodule 

//D触发器
module flop #(parameter WIDTH = 32)
    (input                  clk, 
    input       [WIDTH-1:0] d, 
    output reg [WIDTH-1:0] q);
    always @(posedge clk)
        q <= d;
endmodule

//带使能端的D触发器
module flopen #(parameter WIDTH = 32)
    (input                  clk, en,
    input       [WIDTH-1:0] d, 
    output reg [WIDTH-1:0] q);
    always @(posedge clk)
        if (en) q <= d;
endmodule

//带使能端和复位信号的D触发器
module flopenr #(parameter WIDTH = 32)
    (input                  clk, reset, en,
    input       [WIDTH-1:0] d, 
    output reg [WIDTH-1:0] q);
    always @(posedge clk)
        if      (reset) q <= 0;
        else if (en)    q <= d;
endmodule

module PC(
        input clk,
        input reset,
        input pcen,
        input [15:0] next_ins,
        output [15:0] ins);
    flopenr #(16) pc(clk,reset,pcen,next_ins,ins);
endmodule

//二选一
module mux2 #(parameter WIDTH = 32)
    (input  [WIDTH-1:0] d0, d1, 
    input               s, 
    output [WIDTH-1:0]  y);
    assign y = s ? d1 : d0; 
endmodule

//四选一
module mux4 #(parameter WIDTH = 32)
    (input       [WIDTH-1:0] d0, d1, d2, d3,
    input        [1:0]       s, 
    output reg  [WIDTH-1:0] y);
    always @(*)
        case(s)
            2'b00: y <= d0;
            2'b01: y <= d1;
            2'b10: y <= d2;
            2'b11: y <= d3;
        endcase
endmodule

//符号扩展
module signextend (
    input [15  : 0] src,
    output [31 : 0] res
    );

    assign res = {{16{src[15]}}, src};

endmodule 

//左移两位
module leftshift2 #(parameter WIDTH = 32) (
    input [WIDTH-1  : 0] src,
    output [WIDTH-1 : 0] out
    );

    assign out = src << 2;

endmodule 

module key_fangdou(clk,key_in,key_out);
	parameter SAMPLE_TIME = 1000000;
	input clk;
	input key_in;
	output key_out;

	reg [21:0] count_low;
	reg [21:0] count_high;

	reg key_out_reg;

	always @(posedge clk)
		if(key_in ==1'b0)
			count_low <= count_low + 1;
		else
			count_low <= 0;

	always @(posedge clk)
		if(key_in ==1'b1)
			count_high <= count_high + 1;
		else
			count_high <= 0;

	always @(posedge clk)
		if(count_high == SAMPLE_TIME)
			key_out_reg <= 1;
		else if(count_low == SAMPLE_TIME)
			key_out_reg <= 0;

	assign key_out = key_out_reg;
endmodule