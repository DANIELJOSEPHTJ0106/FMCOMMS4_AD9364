// `timescale 1ns / 1ps

// module axi_pattern_gen (
//     input  wire         aclk,           // 61.44 MHz
//     input  wire         aresetn,

//     // AXI Stream Master Output
//     output wire [7:0]   m_axis_tdata,   // Bit 0 is the PRBS Data
//     output wire [12:0]  m_axis_tuser,   // Index 0-6399
// //    output wire         m_axis_tvalid,  // Always 1
//     output reg         m_axis_tvalid,  // Always 1

//     input  wire         m_axis_tready,  // Backpressure input
//     output wire         m_axis_tlast    // Always 0
// );

//     // Parameters
//     localparam [12:0] SAMPLES_PER_BIT = 6400;
    
//     // PRBS-15 Polynomial: x^15 + x^14 + 1
//     // Cycle length: 32,767 bits before repeating
//     reg [14:0] lfsr; 
    
//     reg [12:0] sample_counter;

//     // --- State Machine ---
//     always @(posedge aclk or negedge aresetn) begin
//         if (!aresetn) begin
//             sample_counter <= 0;
//             // LFSR cannot be all zeros, seed with something non-zero
//             lfsr           <= 15'h7FFF; 
//         end else begin
//             if (m_axis_tready) begin
//                 if (sample_counter == SAMPLES_PER_BIT - 1) begin
//                     // --- End of Bit Period: Shift LFSR ---
//                     sample_counter <= 0;
                    
//                     // PRBS-15 Feedback Logic
//                     // New Bit = lfsr[14] XOR lfsr[13]
//                     lfsr <= {lfsr[13:0], lfsr[14] ^ lfsr[13]};
//                     m_axis_tvalid <= 1;
                    
//                 end else begin
//                     // Hold current state
//                     sample_counter <= sample_counter + 1;
//                     m_axis_tvalid <= 1;
//                 end
//             end
//         end
//     end

//     // --- Output Assignments ---
    
//     // We output the MSB of the LFSR as our "Random Bit"
//     // Using the MSB ensures the data changes right after the shift happens.
//     assign m_axis_tdata  = {7'b0000000, lfsr[14]};
    
//     assign m_axis_tuser  = sample_counter;
// //    assign m_axis_tvalid = 1'b1;
//     assign m_axis_tlast  = 1'b0;

// endmodule

`timescale 1ns / 1ps

module axi_pattern_gen (
    input  wire         aclk,           // 61.44 MHz
    input  wire         aresetn,

    // AXI Stream Master Output
    output wire [7:0]   m_axis_tdata,   // Bit 0 is the PRBS Data
    output wire [12:0]  m_axis_tuser,   // Index 0-6399
//    output wire         m_axis_tvalid,  // Always 1
    output reg         m_axis_tvalid,  // Always 1

    input  wire         m_axis_tready,  // Backpressure input
    output wire         m_axis_tlast    // Always 0
);

// Parameters
localparam [12:0] SAMPLES_PER_BIT = 6400;

// PRBS-15 Polynomial: x^15 + x^14 + 1
// Cycle length: 32,767 bits before repeating
// reg [14:0] lfsr; 

reg [12:0] sample_counter;

reg [7:0] temp_cntr;
reg [3:0] bit_cntr;
// --- State Machine ---
always @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
        sample_counter <= 0;
        // LFSR cannot be all zeros, seed with something non-zero
        // lfsr           <= 15'h7FFF; 
    end else begin
        if (m_axis_tready) begin
            if (sample_counter == SAMPLES_PER_BIT - 1) begin
                // --- End of Bit Period: Shift LFSR ---
                sample_counter <= 0;
                
                // PRBS-15 Feedback Logic
                // New Bit = lfsr[14] XOR lfsr[13]
                // lfsr <= {lfsr[13:0], lfsr[14] ^ lfsr[13]};
                m_axis_tvalid <= 1;
                
            end else begin
                // Hold current state
                sample_counter <= sample_counter + 1;
                m_axis_tvalid <= 1;
            end
        end
    end
end

reg [7:0] temporary_data;

always @(posedge aclk or negedge aresetn) begin
     if (!aresetn) begin
        temp_cntr <= 8'd1;
        bit_cntr <= 4'd7;
        // temporary_data <= 8'b10100101;
        temporary_data <= 8'b11001100;

     end
     else begin
        if(m_axis_tready) begin
            if(sample_counter == SAMPLES_PER_BIT - 2) begin
               if(bit_cntr == 4'd0) begin
                    bit_cntr <= 4'd7;
                    temp_cntr <= (temp_cntr == 8'd5) ? 8'd1 : temp_cntr + 8'd1;
               end
               else begin
                    bit_cntr <= bit_cntr - 4'd1;
               end
            end
        end
     end
end
// --- Output Assignments ---

// We output the MSB of the LFSR as our "Random Bit"
// Using the MSB ensures the data changes right after the shift happens.
assign m_axis_tdata  = {7'b0000000, temporary_data[bit_cntr]};

assign m_axis_tuser  = sample_counter;
assign m_axis_tlast  = 1'b0;

endmodule