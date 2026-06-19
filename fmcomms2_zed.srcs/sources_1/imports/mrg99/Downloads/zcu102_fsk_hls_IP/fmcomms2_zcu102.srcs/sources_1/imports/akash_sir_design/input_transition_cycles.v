module input_transition_cycles(
    input wire clock,
    input wire reset_n,

    input wire [7:0] bsync_dat_o,
    input wire bsync_val_o,

    output wire [15:0] ila_alt_trnstion_cycle_count,
    output wire ila_alt_trnstion_cycle_val,
    output wire ila_alt_trnstion_cycle_ovrflw
);

reg [15:0] transition_cycle_count [0:127];
reg [7:0] array_index;
reg cycle_count_overflow [0:127];

reg [15:0] cycle_counter;

reg [7:0] previous_data;

always @(posedge clock or negedge reset_n) begin
    if(!reset_n) begin
        previous_data <= 8'd0;
    end
    else begin
        if(bsync_val_o) begin
            previous_data <= bsync_dat_o;
        end
    end
end

wire change_detected;
assign change_detected = (bsync_dat_o != previous_data) && bsync_val_o;

wire near_of;
assign near_of = (cycle_counter >= 16'd99); // 16'hFFFE

reg ovrflow_det;

reg nrd_wr;
reg [15:0] r_ila_alt_trnstion_cycle_count;
reg r_ila_alt_trnstion_cycle_ovrflw;
reg r_ila_alt_trnstion_cycle_val;

integer i;
always @(posedge clock or negedge reset_n) begin
    if(!reset_n) begin
        cycle_counter <= 16'd0;
        array_index <= 8'd0;
        nrd_wr <= 1'b0;
        ovrflow_det <= 1'b0;
        r_ila_alt_trnstion_cycle_count <= 16'd0;
        r_ila_alt_trnstion_cycle_ovrflw <= 1'b0;
        r_ila_alt_trnstion_cycle_val <= 1'b0;
        for(i = 0; i < 128; i = i + 1) begin
            transition_cycle_count[i] <= 16'd0;
            cycle_count_overflow[i] <= 1'b0;
        end
    end
    else begin
        if(!nrd_wr) begin
            r_ila_alt_trnstion_cycle_val <= 1'b0;
            if(bsync_val_o) begin
                if(change_detected) begin
                    cycle_counter <= 16'd0;
                    array_index <= array_index + 8'd1;
                    transition_cycle_count[array_index] <= cycle_counter;
                    cycle_count_overflow[array_index] <= ovrflow_det;
                    ovrflow_det <= 1'b0;
                end
                else begin
                    cycle_counter <= cycle_counter + 16'd1;
                    if(near_of) begin
                        ovrflow_det <= 1'b1;
                    end
                end
            end
            else begin
                if(array_index == 8'd127) begin
                    nrd_wr <= 1'b1;
                    array_index <= 8'd0;
                end
            end
        end
        else begin
            r_ila_alt_trnstion_cycle_count <= transition_cycle_count[array_index];
            r_ila_alt_trnstion_cycle_ovrflw <= cycle_count_overflow[array_index];
            r_ila_alt_trnstion_cycle_val <= 1'b1;
            if(array_index == 8'd127) begin
                array_index <= 8'd0;
                nrd_wr <= 1'b0;
            end
            else begin
                nrd_wr <= 1'b1;
                array_index <= array_index + 8'd1;
            end
        end
    end
end

assign ila_alt_trnstion_cycle_count = r_ila_alt_trnstion_cycle_count;
assign ila_alt_trnstion_cycle_ovrflw = r_ila_alt_trnstion_cycle_ovrflw;
assign ila_alt_trnstion_cycle_val = r_ila_alt_trnstion_cycle_val;

endmodule