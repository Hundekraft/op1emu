# Audio input from a WAV file (OP1EMU_AUDIO_IN)

Plays a WAV file into the codec's ADC, so input engines (vocoder, pitch correction, amp) can be run natively.

| Variable | Meaning |
|---|---|
| `OP1EMU_AUDIO_IN=file.wav` | 16-bit PCM, mono or stereo, at the SPORT frame rate (44,100 Hz by default; `OP1EMU_SPORT_HZ` changes it and the file must match). Anything else: error on stderr, exit 2 at start. Stereo is averaged to mono. |
| `OP1EMU_AUDIO_IN_START=sec` | Guest time (seconds, may be fractional) of the file's first frame. Default 0. The guest clock is the SPORT clock shared with the output dump: output dump sample n of the TX stream is guest time `tx_start + n / rate`; `tx_start` is the `guest_ns` in the "dumping TX samples" log line. |
| `OP1EMU_AUDIO_IN_LOOP=1` | Repeat the file (period = file length, phase fixed by START). Default: silence after the end (and before START). |

Path: SPORT0 RX (DMA2), the only RX in use. SPORT0 is I2S with 32-bit words and a stereo frame sync, so each frame is
two int32 words (L, R). R246 sums the two words and keeps the high half as the mono input sample
(`input_mono_buf`). The option writes `sample * 32768` (the 16-bit value in the high half, shifted down one bit, so
L + R is the original level) into both words. The codec is not modelled: the mic and line choices only change codec
registers (and the mic boost of the firmware, +10 dB when set), not this data, so both selections see the file. Without
the option RX is silence, as before. Input source 3 (USB audio) uses a different buffer and is not fed.

Frame k of the RX stream is due at the RX DMA start + k / rate and reads file frame `k + floor(rx_start * rate) - START * rate`.
