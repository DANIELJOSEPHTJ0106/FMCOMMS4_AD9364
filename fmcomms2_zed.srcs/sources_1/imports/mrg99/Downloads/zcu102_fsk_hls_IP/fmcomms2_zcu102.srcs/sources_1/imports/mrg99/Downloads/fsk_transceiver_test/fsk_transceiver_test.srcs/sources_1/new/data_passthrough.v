module data_passthrough #(
    parameter DATA_WIDTH = 16
)(
    input  wire                  clk,
    input  wire                  rst_n,   // Active Low Reset
    
    input  wire [DATA_WIDTH-1:0] data_in,
    input  wire                  valid_in, // Write Enable
    
    output reg  [DATA_WIDTH-1:0] data_out,
    output reg                   valid_out
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_out  <= {DATA_WIDTH{1'b0}};
            valid_out <= 1'b0;
        end 
        else begin
            // 1. Pass the Valid Signal (Pipelined to match data delay)
            valid_out <= valid_in;

            // 2. Data Update Logic
            if (valid_in) begin
                // Update: Capture the new value
                data_out <= data_in;
            end
            // Implicit Else: Hold the previous value
        end
    end

endmodule