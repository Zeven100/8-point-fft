`timescale 1ns / 1ps

module TOP(
input clk , start , rst , output [15:0] real_data_o , img_data_o
    );
    wire tc1 , tc2 , tc3 ;
    wire en1 , en2 , en3 ;
    wire [15:0]rom_out ;
    reg s2p_en;
    wire [2:0] addr , w1 , w2 ;
    wire [15:0] s2p_out [7:0] , fftp_real_out [7:0], fftp_img_out [7:0];
    localparam integer fft_delay = 4 ;
    
    PG pg1(clk , start ,tc1 , en1 ) ;
    Loadable_Counter lc1(clk , en1 , addr , tc1) ;
    ROM rom(clk , en1 , addr , rom_out) ;
    always @(posedge clk)begin
        if(en1)s2p_en <= 1 ;
        else s2p_en <= 0 ;
    end
    S2P s2p(clk , rst , s2p_en , rom_out , s2p_out) ;
    
    PG pg2(clk, tc1 , tc2 , en2) ;
    Loadable_Counter lc2(clk , en2 , w1 , tc2) ;
    
    fft_processor fftp(clk , en2 , rst , s2p_out , fftp_real_out, fftp_img_out) ;
    P2S p2s1(clk, rst , en3 , fftp_real_out , real_data_o) ;
    P2S p2s2(clk, rst , en3 , fftp_img_out , img_data_o) ;
  
    PG pg3(clk, tc2 , tc3 , en3) ;
    Loadable_Counter lc3(clk , en3 , w2 , tc3) ;
  
    
    
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
module Loadable_Counter(input clk , en , output [2:0]addr , output reg tc);
reg [2:0]count = 3'b111 ;
reg en_internal = 0 ;
assign addr = count ;
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

module ROM(input clk,ren , input [2:0]addr , output reg [15:0] data_o);

    (* ram_style = "block" *) reg [15:0] mem [0:7] ;
     initial $readmemh("mem_init.mem", mem);
    
    reg [2:0]counter = 3'b111 ;
    reg en_internal = 0 ;
    always @(posedge clk )begin
        if(ren || en_internal)begin
        
         data_o <= mem[addr] ;
         if(counter == 0)begin
            en_internal <= 0 ;
            counter <= 3'b111 ;
         end
         else begin
            counter <= counter- 1 ;
            en_internal <= 1 ;
         end
        end
        else begin
        data_o <= 0 ;
        end
    end

endmodule

/////////////////S2P
module S2P(input clk , rst , en , input [15:0] data_in , output reg [15:0]data_out[7:0]) ;
    reg en_internal ;
    reg [2:0]counter = 3'b111 ;
    always @(posedge clk)begin
        if(rst)begin
            counter <= 3'b111 ;
            for(int i = 0;  i < 8 ; i++)begin
                data_out[i] <= 0 ;
            end
        end
        else begin
            if(en || en_internal)begin
                if(counter == 0)begin
                    en_internal <= 0 ;
                    counter <= 3'b111 ;
                end
                else begin
                    en_internal <= 1 ;
                    counter <= counter - 1 ;
                end
                data_out[0] <= data_in ;
                for(int i = 1;  i < 8 ; i++)begin
                    data_out[i] <= data_out[i-1] ;
                end    
            end
            else begin end
        end
    end

endmodule/////////////////S2P


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

module delay(input clk , input [15:0]in , output reg [15:0] out);
    always@(posedge clk)begin
        out <= in ;
    end
endmodule



module fft_processor (input clk , start , rst , input signed [15:0] data_in [7:0] , output reg signed [15:0] real_out [7:0] , output reg [15:0] img_out [7:0]) ;
    //// just using a delay for now 
    reg en_internal =  0; 
    reg [2:0] counter = 2'b11 ;
    
    
    //////////////////////////stage1 
    wire [15:0] stage1_o[7:0] ;
    BF1 stage1_1(clk , data_in[0] , data_in[4] , stage1_o[0] , stage1_o[1]) ;
    BF1 stage1_2(clk , data_in[2] , data_in[6] , stage1_o[2] , stage1_o[3]) ;
    BF1 stage1_3(clk , data_in[1] , data_in[5] , stage1_o[4] , stage1_o[5]) ;
    BF1 stage1_4(clk , data_in[3] , data_in[7] , stage1_o[6] , stage1_o[7]) ;
    
    ///////////////////////////stage2
    wire [15:0] stage2_o[11:0] ;
    BF1 stage2_1(clk , stage1_o[0], stage1_o[2], stage2_o[0] , stage2_o[1]);
    BF2 stage2_2(clk , stage1_o[1], stage1_o[3], stage2_o[2] , stage2_o[3] , stage2_o[4] , stage2_o[5]) ;
    BF1 stage2_3(clk , stage1_o[4], stage1_o[6], stage2_o[6] , stage2_o[7]);
    BF2 stage2_4(clk , stage1_o[5], stage1_o[7], stage2_o[8] , stage2_o[9] , stage2_o[10] , stage2_o[11]) ;
    
    ////////////////////////////stage3
    wire [15:0] stage3_o[11:0] ;
    delay stage3_1(clk , stage2_o[0] , stage3_o[0]);
    delay stage3_2(clk , stage2_o[1] , stage3_o[1]);
    delay stage3_3(clk , stage2_o[2] , stage3_o[2]);
    delay stage3_4(clk , stage2_o[3] , stage3_o[3]);
    delay stage3_5(clk , stage2_o[4] , stage3_o[4]);
    delay stage3_6(clk , stage2_o[5] , stage3_o[5]);
    delay stage3_7(clk , stage2_o[6] , stage3_o[6]);
    delay stage3_8(clk , stage2_o[7] , stage3_o[7]);
    CMX1 cmx1(clk , stage2_o[8] , stage2_o[9] , stage3_o[8] , stage3_o[9]);
    CMX2 cmx2(clk , stage2_o[10] , stage2_o[11] , stage3_o[10] , stage3_o[11]);
    
    //////////////////////////////stage4
    
    wire [15:0] stage4_real_o[7:0] , stage4_imag_o[7:0];
    ///////////////////real part
    BF1 stage4_real_1(clk , stage3_o[0] , stage3_o[6], stage4_real_o[0] , stage4_real_o[4]);
    BF1 stage4_real_2(clk , stage3_o[2], stage3_o[8], stage4_real_o[1] , stage4_real_o[5]) ;
    BF2 stage4_real_3(clk , stage3_o[1], stage3_o[7], stage4_real_o[2] , stage4_imag_o[2] , stage4_real_o[6] , stage4_imag_o[6]);
    BF1 stage4_real_4(clk , stage3_o[4], stage3_o[10], stage4_real_o[3] , stage4_real_o[7]) ;
    
    ///////////////////imaginary part
    assign stage4_imag_o[0] = 0 ;
    assign stage4_imag_o[4] = 0 ;
    BF1 stage4_imag_2(clk , stage3_o[3], stage3_o[9], stage4_imag_o[1] , stage4_imag_o[5]) ;
    BF1 stage4_imag_4(clk , stage3_o[5], stage3_o[11], stage4_imag_o[3] , stage4_imag_o[7]) ;
   
    
    always @(posedge clk )begin
        if(rst)begin
            en_internal <=  0 ; 
            counter <= 0  ;
            for(int i = 0 ; i <  8 ; i++)begin
                real_out[i] <= 0 ;
                img_out[i] <= 0 ;
            end
        end
        else begin
            for(int i = 0 ; i <  8 ; i++)begin
                real_out[i] <= stage4_real_o[i] ;
                img_out[i] <= stage4_imag_o[i] ;
            end
            
        end
    end

endmodule

module P2S(input clk , rst , en , input [15:0] data_in [7:0] , output reg [15:0] data_out );
    reg [2:0]count = 3'b111 ;
    reg en_internal = 0 ;
    always @(posedge clk)begin
        if(rst)begin
            data_out <= 0 ;
            count <= 0 ;
        end
        else if(en || en_internal )begin
            if(count == 0)begin
                en_internal <= 0 ;
                count <= 3'b111 ;
                data_out <= data_in[7-count] ;
            end
            else begin
                en_internal <= 1 ;
                count <= count - 1 ;
                data_out <= data_in[7-count] ;
            end
        end
        else begin
        count <= 3'b111 ;
        data_out <= 0 ;
        end
    end
endmodule

module tb ;
    reg clk , start ,rst ;
    wire [15:0] real_data_o , img_data_o;
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
        $dumpfile("tb.vcd") ;
        $dumpvars ;
        #400;$finish;
    end
endmodule

