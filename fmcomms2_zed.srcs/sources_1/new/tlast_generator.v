`timescale 1ns / 1ps

module tlast_generator #(
    // Parameterize the packet length
    parameter PACKET_LENGTH = 1024
)(
    input  wire clk,
    input  wire rst_n,       // Active-low asynchronous reset

    // Upstream (Input from Data Source)
    input  wire tvalid_in,
    output wire tready_out,  // Sent back to the data source

    // Downstream (Output to Data Destination)
    output wire tvalid_out,
    input  wire tready_in,   // Coming from the data destination
    output wire tlast_out
);

    // Calculate the maximum count value (0-indexed)
    localparam COUNT_MAX = PACKET_LENGTH - 1;
    
    // Counter register
    reg [$clog2(PACKET_LENGTH)-1 : 0] beat_count;

    // --- Passthrough Assignments ---
    // Pass the valid signal forward and the ready signal backward
    assign tvalid_out = tvalid_in;
    assign tready_out = tready_in;

    // A valid transfer only occurs when BOTH valid and ready are high
    wire transfer_en = tvalid_in && tready_in;

    // --- Counter Logic ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            beat_count <= 0;
        end else if (transfer_en) begin
            // Reset counter when the packet ends, otherwise increment
            if (beat_count == COUNT_MAX) begin
                beat_count <= 0;
            end else begin
                beat_count <= beat_count + 1;
            end
        end
    end

    // --- TLAST Generation ---
    assign tlast_out = (beat_count == COUNT_MAX - 1);

endmodule