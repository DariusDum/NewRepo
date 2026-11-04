`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// audio_jack_live
//
// Live audio from 128-bit blocks (4 x 32-bit CIC samples, decrypted) to the
// Nexys A7 mono jack. Drop-in replacement for audio_jack_out: same ports, but
//   - NO "play faster / slower" (no rate trim)
//   - the sample rate is computed exactly like pdm_clk_gen does, so the jack
//     plays at exactly the rate the microphone records (same 50 MHz clock,
//     same divider -> they can never drift apart)
//
// Three simple rules for the buffer (the external AXI4-Stream FIFO):
//   1. Start: wait until PREFILL blocks are buffered (~13 ms), then play
//      one sample per sample tick, forever.
//   2. Empty (packet from the PC late): hold the last sample (= silence)
//      and continue as soon as data is there again. No long re-buffering.
//   3. Too far behind (more than MAX_LAG blocks buffered, ~52 ms): throw
//      away blocks down to PREFILL -> jump back to live.
//
//   FIFO --> [4-sample block register] --> DC blocker --> gain + saturate
//        --> [test tone] --> 2nd-order sigma-delta --> AUD_PWM (open drain)
//
// Sample order in the block (switches, as before):
//   word_order = 0 : first sample in [127:96]     = 1 : first sample in [31:0]
//   byte_swap  = 1 : reverse the 4 bytes of each sample
//////////////////////////////////////////////////////////////////////////////////

module audio_jack_live #(
    parameter int CLK_HZ   = 50_000_000,
    parameter int PDM_FREQ = 2_400_000,    // SAME value as pdm_microphone's PDM_FREQ
    parameter int CIC_R    = 64,           // CIC decimation
    parameter int PREFILL  = 128,          // blocks before playing starts (128 = 13 ms)
    parameter int MAX_LAG  = 512,          // blocks; above this, jump back to PREFILL (52 ms)
                                           // must be < FIFO depth - one packet
    parameter int LEVEL_W  = 32,           // width of fifo_level
    parameter bit RZ_MODE  = 1'b0,         // 1 = return-to-zero pin coding (less hiss)
    parameter int DC_K     = 10,           // DC blocker corner ~ fs/(2*pi*2^DC_K)
    parameter int TONE_HZ  = 500
) (
    input  logic               clk,
    input  logic               rst,

    // AXI4-Stream slave - from the FIFO IP (m_axis)
    input  logic [127:0]       s_axis_tdata,
    input  logic               s_axis_tvalid,
    output logic               s_axis_tready,
    input  logic [LEVEL_W-1:0] fifo_level,      // FIFO IP axis_wr/rd_data_count (blocks)

    // switches (synchronised inside)
    input  logic [4:0]         gain_shift,      // smaller = louder
    input  logic               test_tone,
    input  logic               word_order,
    input  logic               byte_swap,

    // audio jack
    output wire                AUD_PWM,
    output logic               AUD_SD,

    // LEDs
    output logic               fifo_underflow,  // sticky: a packet came late (rule 2)
    output logic               clip,            // ~0.1 s after any saturation
    output logic               playing,         // prefill done
    output logic               jumped           // sticky: latency got too big (rule 3)
);

    // same arithmetic as pdm_clk_gen: CLK_DIVIDE = CLK/PDM, M_CLK toggles every CLK_DIVIDE/2
    localparam int MOD_DIV    = 2 * ((CLK_HZ / PDM_FREQ) / 2);   // 50M/2.4M -> 20 clocks
    localparam int SAMPLE_DIV = CIC_R * MOD_DIV;                  // 1280 clocks = 39.06 kHz

    // ------------------------------------------------------------------
    // switches
    // ------------------------------------------------------------------
    logic [4:0] shift_q1, shift_q2;
    logic       tone_q1, tone_q2, word_q1, word_q2, swap_q1, swap_q2;

    always_ff @(posedge clk) begin
        shift_q1 <= gain_shift;  shift_q2 <= shift_q1;
        tone_q1  <= test_tone;   tone_q2  <= tone_q1;
        word_q1  <= word_order;  word_q2  <= word_q1;
        swap_q1  <= byte_swap;   swap_q2  <= swap_q1;
    end

    // ------------------------------------------------------------------
    // fixed sample tick: exactly one per CIC output period
    // ------------------------------------------------------------------
    localparam int TW = $clog2(SAMPLE_DIV);
    logic [TW-1:0] tick_cnt;
    logic          tick;

    always_ff @(posedge clk) begin
        if (rst) begin
            tick_cnt <= '0;
            tick     <= 1'b0;
        end else begin
            tick     <= (tick_cnt == TW'(SAMPLE_DIV - 1));
            tick_cnt <= (tick_cnt == TW'(SAMPLE_DIV - 1)) ? '0 : tick_cnt + 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // buffer rules
    // ------------------------------------------------------------------
    logic flushing;
    logic blk_vld;               // block register holds samples (declared here, used below)

    always_ff @(posedge clk) begin
        if (rst) begin
            playing        <= 1'b0;
            flushing       <= 1'b0;
            jumped         <= 1'b0;
            fifo_underflow <= 1'b0;
        end else begin
            // rule 1: start once
            if (!playing && fifo_level >= LEVEL_W'(PREFILL)) playing <= 1'b1;
            // rule 3: too far behind -> drop down to PREFILL
            if (playing && !flushing && fifo_level > LEVEL_W'(MAX_LAG)) begin
                flushing <= 1'b1;
                jumped   <= 1'b1;
            end
            if (flushing && fifo_level <= LEVEL_W'(PREFILL)) flushing <= 1'b0;
            // rule 2: just note it, playback simply holds
            if (tick && playing && !blk_vld) fifo_underflow <= 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // 4-sample block register
    // ------------------------------------------------------------------
    logic [127:0] blk;
    logic [1:0]   idx;
    logic         take;

    assign s_axis_tready = !blk_vld || flushing;      // while flushing: drop blocks
    assign take          = tick && playing && blk_vld && !flushing;

    always_ff @(posedge clk) begin
        if (rst) begin
            blk     <= '0;
            blk_vld <= 1'b0;
            idx     <= '0;
        end else if (flushing) begin
            blk_vld <= 1'b0;
            idx     <= '0;
        end else if (!blk_vld) begin
            if (s_axis_tvalid) begin
                blk     <= s_axis_tdata;
                blk_vld <= 1'b1;
                idx     <= '0;
            end
        end else if (take) begin
            idx <= idx + 1'b1;
            if (idx == 2'd3) blk_vld <= 1'b0;
        end
    end

    logic [1:0]  slot;
    logic [31:0] word, word_sw;

    assign slot    = word_q2 ? idx : 2'd3 - idx;
    assign word    = blk[32*slot +: 32];
    assign word_sw = swap_q2 ? {word[7:0], word[15:8], word[23:16], word[31:24]} : word;

    logic signed [31:0] x;
    logic               x_vld;

    always_ff @(posedge clk) begin
        if (rst) begin
            x     <= '0;
            x_vld <= 1'b0;
        end else begin
            x_vld <= take;
            if (take) x <= word_sw;
        end
    end

    // ------------------------------------------------------------------
    // DC blocker (unchanged)
    // ------------------------------------------------------------------
    localparam int AccW = 32 + DC_K + 1;

    logic signed [AccW-1:0] dc_acc, dc_full;
    logic signed [33:0]     dc, y;
    logic                   y_vld, dc_init;

    assign dc_full = dc_acc >>> DC_K;
    assign dc      = dc_full[33:0];

    always_ff @(posedge clk) begin
        if (rst) begin
            dc_acc  <= '0;
            dc_init <= 1'b0;
            y       <= '0;
            y_vld   <= 1'b0;
        end else begin
            y_vld <= x_vld;
            if (x_vld) begin
                if (!dc_init) begin
                    dc_acc  <= AccW'(x) <<< DC_K;
                    dc_init <= 1'b1;
                    y       <= '0;
                end else begin
                    dc_acc  <= dc_acc + AccW'(x) - AccW'(dc);
                    y       <= 34'(x) - dc;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // gain + saturation (unchanged)
    // ------------------------------------------------------------------
    logic signed [33:0] y_sh;
    logic signed [15:0] s_mic;
    logic               sat;

    assign y_sh = y >>> shift_q2;

    always_comb begin
        sat = 1'b1;
        if (y_sh > 34'sd32767)        s_mic = 16'sh7FFF;
        else if (y_sh < -34'sd32768)  s_mic = 16'sh8000;
        else begin
            s_mic = y_sh[15:0];
            sat   = 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // test tone (unchanged)
    // ------------------------------------------------------------------
    localparam longint TONE_INC = (longint'(TONE_HZ) * SAMPLE_DIV * (64'd1 << 32)) / longint'(CLK_HZ);

    logic [31:0]        phase;
    logic [15:0]        tri_u;
    logic signed [15:0] s_tone;

    always_ff @(posedge clk) begin
        if (rst)       phase <= '0;
        else if (tick) phase <= phase + 32'(TONE_INC);
    end

    assign tri_u  = phase[31] ? ~phase[30:15] : phase[30:15];
    assign s_tone = $signed(tri_u ^ 16'h8000) >>> 2;

    // ------------------------------------------------------------------
    // output sample (held between ticks, and held when a packet is late)
    // ------------------------------------------------------------------
    localparam int CLIP_HOLD = CLK_HZ / 10;
    logic signed [15:0]           sample;
    logic [$clog2(CLIP_HOLD)-1:0] clip_cnt;

    always_ff @(posedge clk) begin
        if (rst) begin
            sample   <= '0;
            clip_cnt <= '0;
        end else begin
            if (tone_q2) begin
                if (tick) sample <= s_tone;
            end else if (y_vld) begin
                sample <= s_mic;
            end
            if (y_vld && sat && !tone_q2) clip_cnt <= $bits(clip_cnt)'(CLIP_HOLD - 1);
            else if (clip_cnt != 0)       clip_cnt <= clip_cnt - 1'b1;
        end
    end

    assign clip = (clip_cnt != 0);

    // ------------------------------------------------------------------
    // 2nd-order sigma-delta, one bit every MOD_DIV clocks (unchanged)
    // ------------------------------------------------------------------
    localparam int MW = $clog2(MOD_DIV);

    logic [MW-1:0] mod_cnt;
    logic          mod_ce;

    always_ff @(posedge clk) begin
        if (rst) begin
            mod_cnt <= '0;
            mod_ce  <= 1'b0;
        end else if (mod_cnt == MW'(MOD_DIV - 1)) begin
            mod_cnt <= '0;
            mod_ce  <= 1'b1;
        end else begin
            mod_cnt <= mod_cnt + 1'b1;
            mod_ce  <= 1'b0;
        end
    end

    localparam int SW = 24;
    localparam logic signed [SW-1:0] FB = 49152;

    logic signed [SW-1:0] i1, i2, fb, i1_n, i2_n;
    logic                 sd_bit;

    assign sd_bit = ~i2[SW-1];
    assign fb     = sd_bit ? FB : -FB;
    assign i1_n   = i1 + SW'(sample) - fb;
    assign i2_n   = i2 + i1_n - fb;

    always_ff @(posedge clk) begin
        if (rst) begin
            i1 <= '0;
            i2 <= '0;
        end else if (mod_ce) begin
            i1 <= i1_n;
            i2 <= i2_n;
        end
    end

    // pin coding: NRZ, or return-to-zero (fixed pulse in the first half of the slot)
    localparam int RZ_W = MOD_DIV / 2;
    logic pin_q;

    always_ff @(posedge clk) begin
        if (rst)          pin_q <= 1'b0;
        else if (RZ_MODE) pin_q <= sd_bit && (mod_cnt >= MW'(1)) && (mod_cnt <= MW'(RZ_W));
        else              pin_q <= sd_bit;
    end

    assign AUD_PWM = pin_q ? 1'bz : 1'b0;      // open drain
    assign AUD_SD  = 1'b1;

endmodule
