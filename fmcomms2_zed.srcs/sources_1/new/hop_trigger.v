module hop_trigger (
    input  wire clk,        // 100 MHz clock
    input  wire rst,        // active high reset
    output reg  tlast       // 1-cycle pulse every 1ms
);

reg [16:0] counter;   // 17 bits enough for 100000

localparam COUNT_1MS = 17'd99999;

always @(posedge clk) begin
    if (rst) begin
        counter <= 0;
        tlast   <= 0;
    end 
    else begin
        if (counter == COUNT_1MS) begin
            counter <= 0;
            tlast   <= 1'b1;   // TLAST pulse
        end 
        else begin
            counter <= counter + 1;
            tlast   <= 1'b0;
        end
    end
end

endmodule