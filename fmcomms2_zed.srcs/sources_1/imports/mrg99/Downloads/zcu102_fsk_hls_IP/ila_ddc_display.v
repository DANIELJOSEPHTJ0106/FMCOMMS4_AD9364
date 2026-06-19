module ila_ddc_display(
    input wire clock,
    input wire reset_n,

    input wire [31:0] fsk_ddc_out,
    input wire fsk_ddc_val_out,

    output wire [31:0] ila_fsk_ddc_out,
    output wire ila_fsk_ddc_val
);

localparam FIFO_DEPTH = 16'd16384;
// localparam REQ_SAMPS = ;

reg [31:0] fifo_din1;
reg fifo_wr_en1;
reg fifo_rd_en1;
wire [31:0] fifo_dout1;
wire fifo_full1;
wire fifo_empty1;

fifo_generator_1 dut1(
    .clk   (clock),
    .srst  (!reset_n),
    .din   (fifo_din1),
    .wr_en (fifo_wr_en1),
    .rd_en (fifo_rd_en1),
    .dout  (fifo_dout1),
    .full  (fifo_full1),
    .empty (fifo_empty1)
);

reg push_now;
reg [1:0] acc_state1;

reg [31:0] get_data1;
reg get_data1_valid;
reg [15:0] pattern_cntr;

always @(posedge clock or negedge reset_n) begin
    if(!reset_n) begin
        acc_state1 <= 2'd0;
        fifo_din1 <= 8'd0;
        fifo_wr_en1 <= 1'b0;
        fifo_rd_en1 <= 1'b0;
        get_data1 <= 8'd0;
        get_data1_valid <= 1'b0;
        pattern_cntr <= 16'd0;
    end
    else begin
        fifo_wr_en1 <= 1'b0;
        case (acc_state1)
            2'd0: begin
                fifo_rd_en1 <= 1'b0;
                get_data1_valid <= 1'b0;
                get_data1 <= 8'd0;
                if(fsk_ddc_val_out) begin
                    fifo_din1 <= fsk_ddc_out;
                    fifo_wr_en1 <= 1'b1;
                    pattern_cntr <= pattern_cntr + 16'd1;
                end
                if(pattern_cntr == FIFO_DEPTH - 1) begin
                    acc_state1 <= 2'd1;
                end
            end
            2'd1: begin
                pattern_cntr <= 16'd0;
                fifo_rd_en1 <= 1'b1;
                acc_state1 <= 2'd3;
            end
            2'd3: begin
                fifo_rd_en1 <= 1'b1;
                pattern_cntr <= pattern_cntr + 16'd1;
                if(pattern_cntr == 16'd1) begin
                    acc_state1 <= 2'd2;
                    pattern_cntr <= 16'd0;
                end
            end
            2'd2: begin
                fifo_rd_en1 <= 1'b1;
                get_data1_valid <= 1'b1;
                get_data1 <= fifo_dout1;
                if(pattern_cntr == FIFO_DEPTH - 1) begin
                    fifo_rd_en1 <= 1'b0;
                    acc_state1 <= 2'd0;
                    pattern_cntr <= 16'd0;
                end
                else begin
                    pattern_cntr <= pattern_cntr + 16'd1;
                end
            end
            default: acc_state1 <= 2'd0;
        endcase
    end
end

assign ila_fsk_ddc_out = get_data1;
assign ila_fsk_ddc_val = get_data1_valid;

endmodule