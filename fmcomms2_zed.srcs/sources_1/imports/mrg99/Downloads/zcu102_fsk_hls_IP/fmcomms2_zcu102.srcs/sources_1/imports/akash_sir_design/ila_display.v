module ila_display(
    input wire clock_61,
    input wire clock_100,
    input wire reset_n,

    input wire [7:0] pattern_gen_dat_out,
    input wire pattern_gen_val_out,
    input wire [7:0] bsync_dat,
    input wire bsync_val,

    output wire [7:0] ila_pattern_gen_dat,
    output wire ila_pattern_gen_val,
    output wire [7:0] ila_bsync_dat,
    output wire ila_bsync_val
);

localparam FIFO_DEPTH = 16'd512;
// localparam REQ_SAMPS = ;

reg [7:0] fifo_din1;
reg fifo_wr_en1;
reg fifo_rd_en1;
wire [7:0] fifo_dout1;
wire fifo_full1;
wire fifo_empty1;

fifo_generator_0 dut1(
    .wr_clk   (clock_61),
    .rd_clk   (clock_100),
    .rst  (!reset_n),
    .din   (fifo_din1),
    .wr_en (fifo_wr_en1),
    .rd_en (fifo_rd_en1),
    .dout  (fifo_dout1),
    .full  (fifo_full1),
    .empty (fifo_empty1)
);

reg [7:0] fifo_din2;
reg fifo_wr_en2;
reg fifo_rd_en2;
wire [7:0] fifo_dout2;
wire fifo_full2;
wire fifo_empty2;

fifo_generator_0 dut2(
    .wr_clk   (clock_61),
    .rd_clk   (clock_100),
    .rst  (!reset_n),
    .din   (fifo_din2),
    .wr_en (fifo_wr_en2),
    .rd_en (fifo_rd_en2),
    .dout  (fifo_dout2),
    .full  (fifo_full2),
    .empty (fifo_empty2)
);

reg push_now;
reg [1:0] acc_state1;
reg [1:0] acc_state2;

(* ASYNC_REG = "TRUE" *) reg sync_send_done1_0;
(* ASYNC_REG = "TRUE" *) reg sync_send_done1_1;
(* ASYNC_REG = "TRUE" *) reg sync_send_done2_0;
(* ASYNC_REG = "TRUE" *) reg sync_send_done2_1;

reg send_done1;
reg send_done2;

always @(posedge clock_61 or negedge reset_n) begin
    if(!reset_n) begin
        push_now <= 1'b0;
        sync_send_done1_0 <= 1'b0;
        sync_send_done1_1 <= 1'b0;
        sync_send_done2_0 <= 1'b0;
        sync_send_done2_1 <= 1'b0;
    end
    else begin
        if(acc_state2 == 2'd1 && acc_state1 == 2'd1) begin
            push_now <= 1'b1;
        end
        else begin
            push_now <= 1'b0;
        end
        sync_send_done1_0 <= send_done1;
        sync_send_done1_1 <= sync_send_done1_0;
        sync_send_done2_0 <= send_done2;
        sync_send_done2_1 <= sync_send_done2_0;
    end
end

(* ASYNC_REG = "TRUE" *) reg sync_push_now_0;
(* ASYNC_REG = "TRUE" *) reg sync_push_now_1;

always @(posedge clock_100 or negedge reset_n) begin
    if(!reset_n) begin
        sync_push_now_0 <= 1'b0;
        sync_push_now_1 <= 1'b0;
    end
    else begin
        sync_push_now_0 <= push_now;
        sync_push_now_1 <= sync_push_now_0;
    end
end

reg [7:0] get_data1;
reg get_data1_valid;
reg [15:0] pattern_cntr;

reg [7:0] get_data2;
reg get_data2_valid;
reg [15:0] bit_sync_cntr;

always @(posedge clock_61 or negedge reset_n) begin
    if(!reset_n) begin
        acc_state1 <= 2'd0;
        fifo_din1 <= 8'd0;
        fifo_wr_en1 <= 1'b0;
        pattern_cntr <= 16'd0;
    end
    else begin
        fifo_wr_en1 <= 1'b0;
        case (acc_state1)
            2'd0: begin
                if(pattern_gen_val_out) begin
                    fifo_din1 <= pattern_gen_dat_out;
                    fifo_wr_en1 <= 1'b1;
                    pattern_cntr <= pattern_cntr + 16'd1;
                end
                if(pattern_cntr == FIFO_DEPTH - 1) begin
                    acc_state1 <= 2'd1;
                end
            end
            2'd1: begin
                if(push_now) begin
                    pattern_cntr <= 16'd0;
                    acc_state1 <= 2'd2;
                end
            end
            2'd2: begin
                if(sync_send_done1_1) begin
                    acc_state1 <= 2'd0;
                end
            end
            default: acc_state1 <= 2'd0;
        endcase
    end
end

reg [1:0] push_stm1;
reg [15:0] pattern_cntr1;

always @(posedge clock_100 or negedge reset_n) begin
    if(!reset_n) begin
        push_stm1 <= 2'd0;
        send_done1 <= 1'b0;
        fifo_rd_en1 <= 1'b0;
        pattern_cntr1 <= 16'd0;
        get_data1 <= 8'd0;
        get_data1_valid <= 1'b0;
    end
    else begin
        case (push_stm1)
            2'd0: begin
                send_done1 <= 1'b0;
                fifo_rd_en1 <= 1'b0;
                get_data1_valid <= 1'b0;
                get_data1 <= 8'd0;
                if(sync_push_now_1) begin
                    push_stm1 <= 2'd1;
                    fifo_rd_en1 <= 1'b1;
                end
            end
            2'd1: begin
                fifo_rd_en1 <= 1'b1;
                pattern_cntr1 <= pattern_cntr1 + 16'd1;
                if(pattern_cntr1 == 16'd1) begin
                    push_stm1 <= 2'd2;
                    pattern_cntr1 <= 16'd0;
                end
            end
            2'd2: begin
                fifo_rd_en1 <= 1'b1;
                get_data1_valid <= 1'b1;
                get_data1 <= fifo_dout1;
                if(pattern_cntr1 == FIFO_DEPTH - 1) begin
                    fifo_rd_en1 <= 1'b0;
                    push_stm1 <= 2'd3;
                    pattern_cntr1 <= 16'd0;
                    send_done1 <= 1'b1;
                end
                else begin
                    pattern_cntr1 <= pattern_cntr1 + 16'd1;
                end
            end
            2'd3: begin
                pattern_cntr1 <= pattern_cntr1 + 16'd1;
                if(pattern_cntr1 == 16'd1) begin
                    push_stm1 <= 2'd0;
                    pattern_cntr1 <= 16'd0;
                end
            end
            default: push_stm1 <= 2'd0;
        endcase
    end
end

always @(posedge clock_61 or negedge reset_n) begin
    if(!reset_n) begin
        acc_state2 <= 2'd0;
        fifo_din2 <= 8'd0;
        fifo_wr_en2 <= 1'b0;
        bit_sync_cntr <= 16'd0;
    end
    else begin
        fifo_wr_en2 <= 1'b0;
        case (acc_state2)
            2'd0: begin
                if(bsync_val) begin
                    fifo_din2 <= bsync_dat;
                    fifo_wr_en2 <= 1'b1;
                    bit_sync_cntr <= bit_sync_cntr + 16'd1;
                end
                if(bit_sync_cntr == FIFO_DEPTH - 1) begin
                    acc_state2 <= 2'd1;
                end
            end
            2'd1: begin
                if(push_now) begin
                    bit_sync_cntr <= 16'd0;
                    acc_state2 <= 2'd2;
                end
            end
            2'd2: begin
                if(sync_send_done2_1) begin
                    acc_state2 <= 2'd0;
                end
            end
            default: acc_state2 <= 2'd0;
        endcase
    end
end

reg [1:0] push_stm2;
reg [15:0] pattern_cntr2;

always @(posedge clock_100 or negedge reset_n) begin
    if(!reset_n) begin
        push_stm2 <= 2'd0;
        send_done2 <= 1'b0;
        fifo_rd_en2 <= 1'b0;
        pattern_cntr2 <= 16'd0;
        get_data2 <= 8'd0;
        get_data2_valid <= 1'b0;
    end
    else begin
        case (push_stm2)
            2'd0: begin
                send_done2 <= 1'b0;
                fifo_rd_en2 <= 1'b0;
                get_data2_valid <= 1'b0;
                get_data2 <= 8'd0;
                if(sync_push_now_1) begin
                    push_stm2 <= 2'd1;
                    fifo_rd_en2 <= 1'b1;
                end
            end
            2'd1: begin
                fifo_rd_en2 <= 1'b1;
                pattern_cntr2 <= pattern_cntr2 + 16'd1;
                if(pattern_cntr2 == 16'd1) begin
                    push_stm2 <= 2'd2;
                    pattern_cntr2 <= 16'd0;
                end
            end
            2'd2: begin
                fifo_rd_en2 <= 1'b1;
                get_data2_valid <= 1'b1;
                get_data2 <= fifo_dout2;
                if(pattern_cntr2 == FIFO_DEPTH - 1) begin
                    fifo_rd_en2 <= 1'b0;
                    push_stm2 <= 2'd3;
                    pattern_cntr2 <= 16'd0;
                    send_done2 <= 1'b1;
                end
                else begin
                    pattern_cntr2 <= pattern_cntr2 + 16'd1;
                end
            end
            2'd3: begin
                pattern_cntr2 <= pattern_cntr2 + 16'd1;
                if(pattern_cntr2 == 16'd1) begin
                    push_stm2 <= 2'd0;
                    pattern_cntr2 <= 16'd0;
                end
            end
            default: push_stm2 <= 2'd0;
        endcase
    end
end

assign ila_pattern_gen_dat = get_data1;
assign ila_pattern_gen_val = get_data1_valid;
assign ila_bsync_dat = get_data2;
assign ila_bsync_val = get_data2_valid;

endmodule