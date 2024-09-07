`default_nettype none

module CDC_FIFO_Buffer
#(
    parameter WORD_WIDTH        = 0,
    parameter DEPTH             = 0,
    parameter RAMSTYLE          = "",
    parameter CIRCULAR_BUFFER   = 0,         // non-zero to enable
    parameter CDC_EXTRA_STAGES  = 0
)
(
    input   wire                        input_clock,
    input   wire                        input_clear,
    input   wire                        input_valid,
    output  reg                         input_ready,
    input   wire    [WORD_WIDTH-1:0]    input_data,

    input   wire                        output_clock,
    input   wire                        output_clear,
    output  wire                        output_valid,
    input   wire                        output_ready,
    output  wire    [WORD_WIDTH-1:0]    output_data
);

    initial begin
        input_ready = 1'b1; // Empty at start, so accept data
    end

 `include "clog2_function.vh"

    localparam WORD_ZERO    = {WORD_WIDTH{1'b0}};

    localparam ADDR_WIDTH   = clog2(DEPTH);
    localparam ADDR_ONE     = {{ADDR_WIDTH-1{1'b0}},1'b1};
    localparam ADDR_ZERO    = {ADDR_WIDTH{1'b0}};
    localparam ADDR_LAST    = DEPTH-1;

 reg                     buffer_wren = 1'b0;
    wire [ADDR_WIDTH-1:0]   buffer_write_addr;

    reg                     buffer_rden = 1'b0;
    wire [ADDR_WIDTH-1:0]   buffer_read_addr;

    RAM_Simple_Dual_Port_Dual_Clock
    #(
        .WORD_WIDTH     (WORD_WIDTH),
        .ADDR_WIDTH     (ADDR_WIDTH),
        .DEPTH          (DEPTH),
        .RAMSTYLE       (RAMSTYLE),
        .USE_INIT_FILE  (0),
        .INIT_FILE      (),
        .INIT_VALUE     (WORD_ZERO)
    )
    buffer
    (
        .write_clock    (input_clock),
        .wren           (buffer_wren),
        .write_addr     (buffer_write_addr),
        .write_data     (input_data),

        .read_clock     (output_clock),
        .rden           (buffer_rden),
        .read_addr      (buffer_read_addr),
        .read_data      (output_data)
    );

 reg increment_buffer_write_addr = 1'b0;
    reg load_buffer_write_addr      = 1'b0;

    Counter_Binary
    #(
        .WORD_WIDTH     (ADDR_WIDTH),
        .INCREMENT      (ADDR_ONE),
        .INITIAL_COUNT  (ADDR_ZERO)
    )
    write_address
    (
        .clock          (input_clock),
        .clear          (input_clear),
        .up_down        (1'b0), // 0/1 --> up/down
        .run            (increment_buffer_write_addr),
        .load           (load_buffer_write_addr),
        .load_count     (ADDR_ZERO),
        .carry_in       (1'b0),
        // verilator lint_off PINCONNECTEMPTY
        .carry_out      (),
        .carries        (),
        .overflow       (),
        // verilator lint_on  PINCONNECTEMPTY
        .count          (buffer_write_addr)
    );

    reg increment_buffer_read_addr = 1'b0;
    reg load_buffer_read_addr      = 1'b0;

    Counter_Binary
    #(
        .WORD_WIDTH     (ADDR_WIDTH),
        .INCREMENT      (ADDR_ONE),
        .INITIAL_COUNT  (ADDR_ZERO)
    )
    read_address
    (
        .clock          (output_clock),
        .clear          (output_clear),
        .up_down        (1'b0), // 0/1 --> up/down
        .run            (increment_buffer_read_addr),
        .load           (load_buffer_read_addr),
        .load_count     (ADDR_ZERO),
        .carry_in       (1'b0),
        // verilator lint_off PINCONNECTEMPTY
        .carry_out      (),
        .carries        (),
        .overflow       (),
        // verilator lint_on  PINCONNECTEMPTY
        .count          (buffer_read_addr)
    );

reg  toggle_buffer_write_addr_wrap_around = 1'b0;
    wire buffer_write_addr_wrap_around;

    Register_Toggle
    #(
        .WORD_WIDTH     (1),
        .RESET_VALUE    (1'b0)
    )
    write_wrap_around_bit
    (
        .clock          (input_clock),
        .clock_enable   (1'b1),
        .clear          (input_clear),
        .toggle         (toggle_buffer_write_addr_wrap_around),
        .data_in        (buffer_write_addr_wrap_around),
        .data_out       (buffer_write_addr_wrap_around)
    );

    reg  toggle_buffer_read_addr_wrap_around = 1'b0;
    wire buffer_read_addr_wrap_around;

    Register_Toggle
    #(
        .WORD_WIDTH     (1),
        .RESET_VALUE    (1'b0)
    )
    read_wrap_around_bit
    (
        .clock          (output_clock),
        .clock_enable   (1'b1),
        .clear          (output_clear),
        .toggle         (toggle_buffer_read_addr_wrap_around),
        .data_in        (buffer_read_addr_wrap_around),
        .data_out       (buffer_read_addr_wrap_around)
    );

 wire [ADDR_WIDTH-1:0]   buffer_write_addr_synced;
    wire                    buffer_write_addr_wrap_around_synced;
    wire                    buffer_write_addr_synced_valid;

    CDC_Word_Synchronizer
    #(
        .WORD_WIDTH             (ADDR_WIDTH + 1),
        .EXTRA_CDC_DEPTH        (CDC_EXTRA_STAGES),
        .OUTPUT_BUFFER_TYPE     ("HALF"), // "HALF", "SKID", "FIFO"
        .OUTPUT_BUFFER_CIRCULAR (0),
        .FIFO_BUFFER_DEPTH      (), // Only for "FIFO"
        .FIFO_BUFFER_RAMSTYLE   ()  // Only for "FIFO"
    )
    write_to_read
    (
        .sending_clock          (input_clock),
        .sending_clear          (input_clear),
        .sending_data           ({buffer_write_addr_wrap_around, buffer_write_addr}),
        .sending_valid          (1'b1),
        // verilator lint_off PINCONNECTEMPTY
        .sending_ready          (),
        // verilator lint_on  PINCONNECTEMPTY

        .receiving_clock        (output_clock),
        .receiving_clear        (output_clear),
        .receiving_data         ({buffer_write_addr_wrap_around_synced, buffer_write_addr_synced}),
        .receiving_valid        (buffer_write_addr_synced_valid),
        .receiving_ready        (buffer_write_addr_synced_valid)
    );

    wire [ADDR_WIDTH-1:0]   buffer_read_addr_synced;
    wire                    buffer_read_addr_wrap_around_synced;
    wire                    buffer_read_addr_synced_valid;

    CDC_Word_Synchronizer
    #(
        .WORD_WIDTH             (ADDR_WIDTH + 1),
        .EXTRA_CDC_DEPTH        (CDC_EXTRA_STAGES),
        .OUTPUT_BUFFER_TYPE     ("HALF"), // "HALF", "SKID", "FIFO"
        .OUTPUT_BUFFER_CIRCULAR (0),
        .FIFO_BUFFER_DEPTH      (), // Only for "FIFO"
        .FIFO_BUFFER_RAMSTYLE   ()  // Only for "FIFO"
    )
    read_to_write
    (
        .sending_clock          (output_clock),
        .sending_clear          (output_clear),
        .sending_data           ({buffer_read_addr_wrap_around, buffer_read_addr}),
        .sending_valid          (1'b1),
        // verilator lint_off PINCONNECTEMPTY
        .sending_ready          (),
        // verilator lint_on  PINCONNECTEMPTY

        .receiving_clock        (input_clock),
        .receiving_clear        (input_clear),
        .receiving_data         ({buffer_read_addr_wrap_around_synced, buffer_read_addr_synced}),
        .receiving_valid        (buffer_read_addr_synced_valid),
        .receiving_ready        (buffer_read_addr_synced_valid)
    );

 reg stored_items_zero = 1'b0;
    reg stored_items_max  = 1'b0;

    always @(*) begin
        stored_items_zero = (buffer_read_addr        == buffer_write_addr_synced) && (buffer_read_addr_wrap_around        == buffer_write_addr_wrap_around_synced);
        stored_items_max  = (buffer_read_addr_synced == buffer_write_addr)        && (buffer_read_addr_wrap_around_synced != buffer_write_addr_wrap_around);
    end

 reg insert = 1'b0;

    always @(*) begin
        input_ready                             = (stored_items_max == 1'b0) || (CIRCULAR_BUFFER != 0);
        insert                                  = (input_valid      == 1'b1) && (input_ready  == 1'b1);

        buffer_wren                             = (insert == 1'b1);
        increment_buffer_write_addr             = (insert == 1'b1);
        load_buffer_write_addr                  = (increment_buffer_write_addr == 1'b1) && (buffer_write_addr == ADDR_LAST [ADDR_WIDTH-1:0]);
        toggle_buffer_write_addr_wrap_around    = (load_buffer_write_addr      == 1'b1);
    end

reg remove_normal           = 1'b0;
    reg remove_circular         = 1'b0;
    reg remove                  = 1'b0;
    reg output_leaving_idle     = 1'b0;
    reg load_output_register    = 1'b0;

    always @(*) begin
        remove_normal                       = (output_valid    == 1'b1) && (output_ready        == 1'b1);
        remove_circular                     = (CIRCULAR_BUFFER !=    0) && (stored_items_max    == 1'b1) && (insert == 1'b1);
        remove                              = (remove_normal   == 1'b1) || (remove_circular     == 1'b1);
        output_leaving_idle                 = (output_valid    == 1'b0) && (stored_items_zero   == 1'b0);
        load_output_register                = (remove          == 1'b1) || (output_leaving_idle == 1'b1);

        buffer_rden                         = (load_output_register       == 1'b1) && (stored_items_zero == 1'b0);
        increment_buffer_read_addr          = (load_output_register       == 1'b1) && (stored_items_zero == 1'b0);
        load_buffer_read_addr               = (increment_buffer_read_addr == 1'b1) && (buffer_read_addr  == ADDR_LAST [ADDR_WIDTH-1:0]);
        toggle_buffer_read_addr_wrap_around = (load_buffer_read_addr      == 1'b1);
    end

  Register
    #(
        .WORD_WIDTH     (1),
        .RESET_VALUE    (1'b0)
    )
    output_data_valid
    (
        .clock          (output_clock),
        .clock_enable   (load_output_register == 1'b1),
        .clear          (output_clear),
        .data_in        (stored_items_zero == 1'b0),
        .data_out       (output_valid)
    );

endmodule