`timescale 1ns / 1ps
module TOP(
    input clk, start, rst,
    output signed [15:0] real_data_o, img_data_o
);
    wire tc1, tc2, tc3;
    wire en1, en2, en3;
    wire signed [15:0] rom_out;
    reg s2p_en1 , s2p_en2;
    wire [3:0] w1, w2;
    wire [3:0] addr ;

    // Declare unpacked arrays as separate signals
    wire signed [15:0] s2p_out0, s2p_out1, s2p_out2, s2p_out3;
    wire signed [15:0] s2p_out4, s2p_out5, s2p_out6, s2p_out7;
    
    wire signed [15:0] fftp_real_out0, fftp_real_out1, fftp_real_out2, fftp_real_out3;
    wire signed [15:0] fftp_real_out4, fftp_real_out5, fftp_real_out6, fftp_real_out7;
    
    wire signed [15:0] fftp_img_out0, fftp_img_out1, fftp_img_out2, fftp_img_out3;
    wire signed [15:0] fftp_img_out4, fftp_img_out5, fftp_img_out6, fftp_img_out7;
    
    localparam integer fft_delay = 4;

    // Control logic
    PG pg1(clk, start, tc1, en1);
    Loadable_Counter lc1(clk, en1, addr, tc1);
    ROM rom(clk, en1, addr, rom_out);

    always @(posedge clk) begin
        if (en1) s2p_en1 <= 1;
        else s2p_en1 <= 0;
    end
    
    

    // S2P instantiation
    S2P s2p(
        clk, rst, s2p_en1 , rom_out
        ,s2p_out0, s2p_out1, s2p_out2, s2p_out3, 
                   s2p_out4, s2p_out5, s2p_out6, s2p_out7
    );

    // Stage 2 PG and Counter
    PG pg2(clk, tc1, tc2, en2);
    Loadable_Counter lc2(clk, en2, w1, tc2);

    // FFT Processor instantiation
    fft_processor fftp(
        .clk(clk), .start(en2), .rst(rst), 
        .data_in0(s2p_out0), .data_in1(s2p_out1), .data_in2(s2p_out2), .data_in3(s2p_out3),
        .data_in4(s2p_out4), .data_in5(s2p_out5), .data_in6(s2p_out6), .data_in7(s2p_out7),
        .real_out0(fftp_real_out0), .real_out1(fftp_real_out1), .real_out2(fftp_real_out2), .real_out3(fftp_real_out3),
        .real_out4(fftp_real_out4), .real_out5(fftp_real_out5), .real_out6(fftp_real_out6), .real_out7(fftp_real_out7),
        .img_out0(fftp_img_out0), .img_out1(fftp_img_out1), .img_out2(fftp_img_out2), .img_out3(fftp_img_out3),
        .img_out4(fftp_img_out4), .img_out5(fftp_img_out5), .img_out6(fftp_img_out6), .img_out7(fftp_img_out7)
    );

    // P2S instances for real and imaginary outputs
    P2S p2s1(
        .clk(clk), .rst(rst), .en(en3),
        .data_in0(fftp_real_out0), .data_in1(fftp_real_out1), .data_in2(fftp_real_out2), .data_in3(fftp_real_out3),
        .data_in4(fftp_real_out4), .data_in5(fftp_real_out5), .data_in6(fftp_real_out6), .data_in7(fftp_real_out7),
        .data_out(real_data_o)
    );

    P2S p2s2(
        .clk(clk), .rst(rst), .en(en3),
        .data_in0(fftp_img_out0), .data_in1(fftp_img_out1), .data_in2(fftp_img_out2), .data_in3(fftp_img_out3),
        .data_in4(fftp_img_out4), .data_in5(fftp_img_out5), .data_in6(fftp_img_out6), .data_in7(fftp_img_out7),
        .data_out(img_data_o)
    );

    // Stage 3 PG and Counter
    PG pg3(clk, tc2, tc3, en3);
    Loadable_Counter lc3(clk, en3, w2, tc3);

endmodule

///////////PG
module PG(input clk , input start , input tc , output reg en);
always @(posedge clk)begin
    if(tc == 1)en <= 0 ;
    else if(start == 1) en <= 1 ;
    else en <= 0 ;
end
endmodule///////////PG

///////////Loadable_Counter
module Loadable_Counter(input clk , en , output [3:0]addr , output reg tc);
reg [2:0]count = 3'b111 ;
reg en_internal = 0 ;
assign addr = {1'b0, count};
always @(posedge clk)begin
    if(en)begin
        
    end
end
always @(posedge clk)begin
    if(en || en_internal)begin
        if(count == 0)begin
            tc <= 1 ;
            en_internal <= 0 ;
        end
        else begin
            en_internal <= 1 ;
            count <= count - 1 ;
        end
    end
    else begin
    count <= 3'b111 ;
    tc <= 0 ;
    end
end
endmodule///////////Loadable_Counter

module BRAM (
    input clk,                  
                        
    input [3:0] read_address,   
    output reg [15:0] read_data );

    // BRAM: 16 locations, 16 bits wide
    reg [15:0] bram [0:15];

    // Initialize memory from file
//    initial $readmemh("mem_init0.mem", bram);
    // hard coding initial values for simulation 
    
    // Test - 1
    initial begin 
        bram[0] = 16'h0000 ; 
        bram[1] = 16'h0100 ; 
        bram[2] = 16'h0200 ; 
        bram[3] = 16'h0300 ; 
        bram[4] = 16'h0400 ; 
        bram[5] = 16'h0500 ; 
        bram[6] = 16'h0600 ; 
        bram[7] = 16'h0700 ;        
    end
   
    
//    // Test - 2
//    initial begin 
//        bram[0] = 16'h0000 ; 
//        bram[1] = 16'h03A6 ; 
//        bram[2] = 16'hF9A6 ; 
//        bram[3] = 16'h07CB ; 
//        bram[4] = 16'h01C2 ; 
//        bram[5] = 16'hFCA3 ; 
//        bram[6] = 16'h0642 ; 
//        bram[7] = 16'hFFD3 ;        
//    end
    integer i;

    always @(posedge clk) begin
        read_data <= bram[read_address];
    end     
endmodule
module ROM (
    input clk,
   input rst , 
    input [3:0] addr,  // 4-bit address (16 locations)
    output signed [15:0] data_o
);

    BRAM bram_inst(clk , addr , data_o) ;
endmodule


module S2P(
    input clk, rst, en, 
    input [15:0] data_in, 
    output reg signed [15:0] data_out0, data_out1, data_out2, data_out3,
                                 data_out4, data_out5, data_out6, data_out7
);
    reg en_internal;
    reg [2:0] counter = 3'b111;

    // Internal shift register to hold 8 values
    reg signed [15:0] shift_reg [7:0];

    always @(posedge clk) begin : block_name
    integer i ;
        if (rst) begin
            counter <= 3'b111;
            en_internal <= 0;
            for ( i = 0; i < 8; i = i + 1) begin
                shift_reg[i] <= 0;
            end
        end 
        else begin
            if (en || en_internal) begin
                if (counter == 0) begin
                    en_internal <= 0;
                    counter <= 3'b111;
                end 
                else begin
                    en_internal <= 1;
                    counter <= counter - 1;
                end

                // Shift operation
                shift_reg[0] <= data_in;
                for ( i = 1; i < 8; i = i + 1) begin 
                    shift_reg[i] <= shift_reg[i-1];
                end    
            end
        end
    end

    // Assigning the outputs explicitly
    always @(*) begin
        data_out0 = shift_reg[0];
        data_out1 = shift_reg[1];
        data_out2 = shift_reg[2];
        data_out3 = shift_reg[3];
        data_out4 = shift_reg[4];
        data_out5 = shift_reg[5];
        data_out6 = shift_reg[6];
        data_out7 = shift_reg[7];
    end

endmodule



//////////BF1
module BF1(input clk , input signed [15:0]in1 , in2 , output reg signed [15:0]out1 , out2) ;
    always@(posedge clk)begin
        out1 <= in1 + in2 ;
        out2 <= in1 - in2 ;
    end
endmodule//////////BF1 

//////////BF2
module BF2(input clk , input signed [15:0]in1 , in2 , output reg signed [15:0]out1 , out2, out3 , out4) ;
    always @(posedge clk)begin
        out1 <= in1 ;
        out2 <= -in2 ;
        out3 <= in1 ;
        out4 <= in2 ;
    end
endmodule///////////BF2

module CMX1(input clk, input signed [15:0]in1 , in2 , output reg signed [15:0]out1 , out2) ;
    wire signed [15:0] sum, diff;
    wire signed [31:0] mult_sum, mult_diff;
    
    // Compute sum and difference
    assign sum = in1 + in2;  // (a + b)
    assign diff = in1 - in2; // (a - b)

    // Multiply by 0.7071 (approx 181 in Q8.8)
    assign mult_sum = sum * 16'sd181;  // 0.7071 * (a + b)
    assign mult_diff = diff * 16'sd181; // 0.7071 * (a - b)

    always @(posedge clk) begin
        // Adjust for fixed-point by shifting right by 8 bits
        out1 <= mult_sum >>> 8; // Real part
        out2 <= -(mult_diff >>> 8); // Imaginary part (negation for -j term)
    end

endmodule

module CMX2(input clk, input signed [15:0]in1 , in2 , output reg signed [15:0]out1 , out2) ;
    wire signed [15:0] sum, diff;
    wire signed [31:0] mult_sum, mult_diff;
    
    // Compute sum and difference
    assign sum = in1 + in2;  // (a + b)
    assign diff = in1 - in2; // (a - b)

    // Multiply by 0.7071 (approx 181 in Q8.8)
    assign mult_sum = sum * 16'sd181;  // 0.7071 * (a + b)
    assign mult_diff = diff * 16'sd181; // 0.7071 * (a - b)

    always @(posedge clk) begin
        // Adjust for fixed-point by shifting right by 8 bits
        out2 <= -(mult_sum >>> 8); //imaginary part
        out1 <= -(mult_diff >>> 8); // Real part
    end

endmodule

module delay(input clk , input signed [15:0]in , output reg signed [15:0] out);
    always@(posedge clk)begin
        out <= in ;
    end
endmodule



module fft_processor (
    input clk, start, rst, 
    input signed [15:0] data_in0, data_in1, data_in2, data_in3,
                        data_in4, data_in5, data_in6, data_in7,
    output reg signed [15:0] real_out0, real_out1, real_out2, real_out3,
                             real_out4, real_out5, real_out6, real_out7,
    output reg signed [15:0] img_out0, img_out1, img_out2, img_out3,
                             img_out4, img_out5, img_out6, img_out7
);

    reg en_internal = 0; 
    reg [2:0] counter = 2'b11;

    //////////////////////////stage1 
    wire [15:0] stage1_o[7:0];

    BF1 stage1_1(clk, data_in0, data_in4, stage1_o[0], stage1_o[1]);
    BF1 stage1_2(clk, data_in2, data_in6, stage1_o[2], stage1_o[3]);
    BF1 stage1_3(clk, data_in1, data_in5, stage1_o[4], stage1_o[5]);
    BF1 stage1_4(clk, data_in3, data_in7, stage1_o[6], stage1_o[7]);

    ///////////////////////////stage2
    wire [15:0] stage2_o[11:0];
    BF1 stage2_1(clk, stage1_o[0], stage1_o[2], stage2_o[0], stage2_o[1]);
    BF2 stage2_2(clk, stage1_o[1], stage1_o[3], stage2_o[2], stage2_o[3], stage2_o[4], stage2_o[5]);
    BF1 stage2_3(clk, stage1_o[4], stage1_o[6], stage2_o[6], stage2_o[7]);
    BF2 stage2_4(clk, stage1_o[5], stage1_o[7], stage2_o[8], stage2_o[9], stage2_o[10], stage2_o[11]);

    ////////////////////////////stage3
    wire [15:0] stage3_o[11:0];
    delay stage3_1(clk, stage2_o[0], stage3_o[0]);
    delay stage3_2(clk, stage2_o[1], stage3_o[1]);
    delay stage3_3(clk, stage2_o[2], stage3_o[2]);
    delay stage3_4(clk, stage2_o[3], stage3_o[3]);
    delay stage3_5(clk, stage2_o[4], stage3_o[4]);
    delay stage3_6(clk, stage2_o[5], stage3_o[5]);
    delay stage3_7(clk, stage2_o[6], stage3_o[6]);
    delay stage3_8(clk, stage2_o[7], stage3_o[7]);
    CMX1 cmx1(clk, stage2_o[8], stage2_o[9], stage3_o[8], stage3_o[9]);
    CMX2 cmx2(clk, stage2_o[10], stage2_o[11], stage3_o[10], stage3_o[11]);

    //////////////////////////////stage4
    wire [15:0] stage4_real_o[7:0], stage4_imag_o[7:0];

    ///////////////////real part
    BF1 stage4_real_1(clk, stage3_o[0], stage3_o[6], stage4_real_o[0], stage4_real_o[4]);
    BF1 stage4_real_2(clk, stage3_o[2], stage3_o[8], stage4_real_o[1], stage4_real_o[5]);
    BF2 stage4_real_3(clk, stage3_o[1], stage3_o[7], stage4_real_o[2], stage4_imag_o[2], stage4_real_o[6], stage4_imag_o[6]);
    BF1 stage4_real_4(clk, stage3_o[4], stage3_o[10], stage4_real_o[3], stage4_real_o[7]);

    ///////////////////imaginary part
    assign stage4_imag_o[0] = 0;
    assign stage4_imag_o[4] = 0;
    BF1 stage4_imag_2(clk, stage3_o[3], stage3_o[9], stage4_imag_o[1], stage4_imag_o[5]);
    BF1 stage4_imag_4(clk, stage3_o[5], stage3_o[11], stage4_imag_o[3], stage4_imag_o[7]);

    always @(posedge clk) begin
        if (rst) begin
            en_internal <= 0; 
            counter <= 0;
            real_out0 <= 0; real_out1 <= 0; real_out2 <= 0; real_out3 <= 0;
            real_out4 <= 0; real_out5 <= 0; real_out6 <= 0; real_out7 <= 0;
            img_out0 <= 0; img_out1 <= 0; img_out2 <= 0; img_out3 <= 0;
            img_out4 <= 0; img_out5 <= 0; img_out6 <= 0; img_out7 <= 0;
        end
        else begin
            real_out0 <= stage4_real_o[0];
            real_out1 <= stage4_real_o[1];
            real_out2 <= stage4_real_o[2];
            real_out3 <= stage4_real_o[3];
            real_out4 <= stage4_real_o[4];
            real_out5 <= stage4_real_o[5];
            real_out6 <= stage4_real_o[6];
            real_out7 <= stage4_real_o[7];

            img_out0 <= stage4_imag_o[0];
            img_out1 <= stage4_imag_o[1];
            img_out2 <= stage4_imag_o[2];
            img_out3 <= stage4_imag_o[3];
            img_out4 <= stage4_imag_o[4];
            img_out5 <= stage4_imag_o[5];
            img_out6 <= stage4_imag_o[6];
            img_out7 <= stage4_imag_o[7];
        end
    end

endmodule

module P2S(
    input clk, rst, en, 
    input signed [15:0] data_in0, data_in1, data_in2, data_in3, 
    input signed [15:0] data_in4, data_in5, data_in6, data_in7, 
    output reg signed [15:0] data_out
);
    reg [2:0] count;
    reg en_internal;

    always @(posedge clk) begin
        if (rst) begin
            data_out <= 0;
            count <= 3'b000;
            en_internal <= 0;
        end 
        else if (en || en_internal) begin
            case (count)
                3'b000: data_out <= data_in0;
                3'b001: data_out <= data_in1;
                3'b010: data_out <= data_in2;
                3'b011: data_out <= data_in3;
                3'b100: data_out <= data_in4;
                3'b101: data_out <= data_in5;
                3'b110: data_out <= data_in6;
                3'b111: data_out <= data_in7;
            endcase
            
            if (count == 3'b111) begin
                en_internal <= 0;
                count <= 3'b000;
            end 
            else begin
                en_internal <= 1;
                count <= count + 1;
            end
        end 
        else begin
            count <= 3'b000;
            data_out <= 0;
            en_internal <= 0;
        end
    end
endmodule

module tb ;
    reg clk , start ,rst ;
    wire signed [15:0] real_data_o , img_data_o;
    TOP top(clk , start ,rst , real_data_o , img_data_o) ;
    always #5 clk = ~clk ;
    initial begin
        rst = 0 ;
        clk = 0 ;
        start = 0 ;
        #10;
        start = 1 ;
        #10 ;
        start = 0 ;
    end 
    initial begin
        #400;$finish;
    end
endmodule
