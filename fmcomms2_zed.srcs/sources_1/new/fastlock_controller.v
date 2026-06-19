//module fastlock_controller (
//    input  wire clk,                // 100 MHz System Clock
//    input  wire rst_n,              // Active-low reset (Matches your block design)
//    output reg  [2:0] fmcomms4_ctrl_in // 3-bit output to AD9364
//);

//    reg [16:0] counter;
//    localparam COUNT_1MS = 17'd99999; // 100,000 clock cycles = 1ms at 100MHz

//    always @(posedge clk or negedge rst_n) begin
//        if (!rst_n) begin
//            // Reset state
//            counter <= 17'd0;
//            fmcomms4_ctrl_in <= 3'b000; 
//        end else begin
//            if (counter == COUNT_1MS) begin
//                // 1ms is reached: Reset timer and change frequency profile
//                counter <= 17'd0;
//                fmcomms4_ctrl_in <= fmcomms4_ctrl_in + 1; 
//            end else begin
//                // Keep counting until 1ms
//                counter <= counter + 1;
//            end
//        end
//    end

//endmodule
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//module fastlock_controller (
//    input  wire clk,                // 100 MHz System Clock
//    input  wire rst_n,              // Active-low reset (Matches your block design)
//    output reg  [2:0] fmcomms4_ctrl_in // 3-bit output to AD9364
//);

//    reg [16:0] counter;
    
//    // 10 Microseconds (10 us) ????????????:
//    // 10 us = 10,000 ns. 
//    // 10,000 ns / 10 ns (Clock Period) = 1000 cycles.
//    // 1000 - 1 = 999.
//    localparam COUNT_10US = 17'd3999; 

//    always @(posedge clk or negedge rst_n) begin
//        if (!rst_n) begin
//            // Reset state
//            counter <= 17'd0;
//            fmcomms4_ctrl_in <= 3'b000; 
//        end else begin
//            if (counter == COUNT_10US) begin
//                // 10us is reached: Reset timer and change frequency profile
//                counter <= 17'd0;
//                fmcomms4_ctrl_in <= fmcomms4_ctrl_in + 1; 
//            end else begin
//                // Keep counting until 10us
//                counter <= counter + 1;
//            end
//        end
//    end

//endmodule
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//module fastlock_controller (
//    input  wire clk,                // 100 MHz System Clock
//    input  wire rst_n,              // Active-low reset
//    output reg  [2:0] fmcomms4_ctrl_in, // 3-bit output to AD9364
//    output wire active_group_flag   // C ?????? ?????? ????????? ????? ???
//);

//    reg [26:0] counter;
    
//    // 40 Microseconds: 40,000 ns / 10 ns = 4000 cycles. (4000 - 1 = 3999)
//    localparam COUNT_40US = 27'd99_999_999; 

//    // ???????? ????????? C ?????? ?????????? MSB ??????????????
//    // 0 = Group A (Profiles 0-3), 1 = Group B (Profiles 4-7)
//    assign active_group_flag = fmcomms4_ctrl_in[2];

//    always @(posedge clk or negedge rst_n) begin
//        if (!rst_n) begin
//            // Reset state
//            counter <= 27'd0;
//            fmcomms4_ctrl_in <= 3'b000; 
//        end else begin
//            if (counter == COUNT_40US) begin
//                // 40us is reached: Reset timer and change frequency profile
//                counter <= 27'd0;
//                fmcomms4_ctrl_in <= fmcomms4_ctrl_in + 1; 
//            end else begin
//                // Keep counting until 40us
//                counter <= counter + 1;
//            end
//        end
//    end

//endmodule
module fastlock_controller (
    input  wire clk,                // 100 MHz System Clock
    input  wire rst_n,              // Active-low reset
    input  wire [31:0] hop_time_cycles, // ????? ????????: C ????? ??????? ?????? ????
    output reg  [2:0] fmcomms4_ctrl_in, // 3-bit output to AD9364
    output wire active_group_flag   // C ?????? ?????? ????????? ???
);

    reg [31:0] counter; // ??????????? ??????? 32 ?????? ?????

    // MSB Signal for C Code
    assign active_group_flag = fmcomms4_ctrl_in[2];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset state
            counter <= 32'd0;
            fmcomms4_ctrl_in <= 3'b000; 
        end else begin
            // 'hop_time_cycles'-? ?????????? ?????????? ???????
            if (counter >= hop_time_cycles) begin
                counter <= 32'd0;
                fmcomms4_ctrl_in <= fmcomms4_ctrl_in + 1; 
            end else begin
                counter <= counter + 1;
            end
        end
    end

endmodule