module complex_mag_sq (
    input  wire        clk,
    input  wire        rst_n,

    // AXI-Stream Slave Interface (Input)
    input  wire [31:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,
    input  wire [9:0]  s_axis_tuser,  // Example 8-bit tuser

    // AXI-Stream Master Interface (Output)
    output reg  [31:0] m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tlast,
    output reg  [9:0]  m_axis_tuser
);

    // Internal signals for sign-extended Real and Imaginary
    wire signed [15:0] real_val = s_axis_tdata[15:0];
    wire signed [15:0] imag_val = s_axis_tdata[31:16];

    // Pipeline registers for synchronization
    reg signed [31:0] p_real_sq, p_imag_sq;
    reg               p1_valid, p2_valid;
    reg               p1_last,  p2_last;
    reg        [9:0]  p1_user,  p2_user;

    // Stage 1: Calculate Squares
    // Squaring a 16-bit signed number results in a 31-bit unsigned value
    always @(posedge clk) begin
        if (!rst_n) begin
            p_real_sq <= 0;
            p_imag_sq <= 0;
            p1_valid  <= 0;
        end else if (s_axis_tready && s_axis_tvalid) begin
            p_real_sq <= real_val * real_val;
            p_imag_sq <= imag_val * imag_val;
            p1_valid  <= 1'b1;
            p1_last   <= s_axis_tlast;
            p1_user   <= s_axis_tuser;
        end else if (m_axis_tready) begin
            p1_valid  <= 1'b0;
        end
    end

    // Stage 2: Summation and Saturation
    wire [32:0] full_sum = p_real_sq + p_imag_sq;

    always @(posedge clk) begin
        if (!rst_n) begin
            m_axis_tdata  <= 0;
            m_axis_tvalid <= 0;
            m_axis_tlast  <= 0;
            m_axis_tuser  <= 0;
        end else if (m_axis_tready) begin
            m_axis_tvalid <= p1_valid;
            m_axis_tlast  <= p1_last;
            m_axis_tuser  <= p1_user;
            
            // Saturation logic: if bit 32 is 1, it overflowed 32 bits
            if (full_sum[32]) begin
                m_axis_tdata <= 32'hFFFFFFFF;
            end else begin
                m_axis_tdata <= full_sum[31:0];
            end
        end
    end

    // Ready signal logic
    assign s_axis_tready = m_axis_tready;

endmodule