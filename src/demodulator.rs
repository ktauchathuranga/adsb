//! Mode S signal demodulation
//!
//! Detects Mode S preambles and demodulates bit streams from magnitude data.

use std::fs::File;
use std::io::{BufReader, Read};

use crossbeam_channel::Sender;
use tracing:: debug;

use crate::config::Config;
use crate::decoder::{self, ModesMessage, MODES_LONG_MSG_BITS, MODES_SHORT_MSG_BITS};
use crate::magnitude::{compute_magnitude_vector, MagnitudeLut};

/// Preamble duration in microseconds
const MODES_PREAMBLE_US: usize = 8;
/// Full message length for buffer sizing
const MODES_FULL_LEN:  usize = MODES_PREAMBLE_US + MODES_LONG_MSG_BITS;
/// Default data buffer length
const MODES_DATA_LEN: usize = 16 * 16384; // 256K

/// Mode S demodulator
pub struct Demodulator {
    config: Config,
    pub mag_lut: MagnitudeLut,
}

impl Demodulator {
    pub fn new(config: Config) -> Self {
        Self {
            config,
            mag_lut: MagnitudeLut::new(),
        }
    }

    /// Process data from a file
    pub fn process_file(&self, filename: &str, tx: &Sender<ModesMessage>) -> std::io::Result<()> {
        let file: Box<dyn Read> = if filename == "-" {
            Box::new(std::io::stdin())
        } else {
            Box::new(File::open(filename)?)
        };

        let mut reader = BufReader::with_capacity(MODES_DATA_LEN, file);

        // Buffer for raw I/Q data with overlap for message detection across reads
        let buffer_len = MODES_DATA_LEN + (MODES_FULL_LEN - 1) * 4;
        let mut data = vec![127u8; buffer_len];

        loop {
            // Move trailing data to beginning (for messages spanning reads)
            let overlap = (MODES_FULL_LEN - 1) * 4;
            data.copy_within(MODES_DATA_LEN.. MODES_DATA_LEN + overlap, 0);

            // Read new data
            let bytes_read = reader.read(&mut data[overlap..overlap + MODES_DATA_LEN])?;

            if bytes_read == 0 {
                if self.config.loop_file && filename != "-" {
                    // Reopen the file to loop
                    drop(reader);
                    if let Ok(file) = File::open(filename) {
                        reader =
                            BufReader::with_capacity(MODES_DATA_LEN, Box::new(file) as Box<dyn Read>);
                        debug!("Looping file");
                        continue;
                    }
                }
                break;
            }

            // Pad with silence if we didn't get a full buffer
            if bytes_read < MODES_DATA_LEN {
                data[overlap + bytes_read..]. fill(127);
            }

            // Convert to magnitude
            let magnitude = compute_magnitude_vector(&data[..overlap + bytes_read], &self.mag_lut);

            // Detect and decode messages
            self.detect_modes(&magnitude, tx);
        }

        Ok(())
    }

    /// Public method for external magnitude data processing
    pub fn detect_modes_external(&self, magnitude: &[u16], tx: &Sender<ModesMessage>) {
        self.detect_modes(magnitude, tx);
    }

    /// Detect Mode S messages in magnitude data. 
    fn detect_modes(&self, magnitude: &[u16], tx: &Sender<ModesMessage>) {
        let mlen = magnitude.len();
        if mlen < MODES_FULL_LEN * 2 {
            return;
        }

        let mut j = 0;
        let mut use_correction = false;

        // Auxiliary buffer for phase correction
        let mut aux = vec![0u16; MODES_LONG_MSG_BITS * 2];

        while j < mlen - MODES_FULL_LEN * 2 {
            // Skip preamble check if we're retrying with phase correction
            if ! use_correction {
                // Check preamble pattern
                let m = magnitude;
                if !(m[j] > m[j + 1]
                    && m[j + 1] < m[j + 2]
                    && m[j + 2] > m[j + 3]
                    && m[j + 3] < m[j]
                    && m[j + 4] < m[j]
                    && m[j + 5] < m[j]
                    && m[j + 6] < m[j]
                    && m[j + 7] > m[j + 8]
                    && m[j + 8] < m[j + 9]
                    && m[j + 9] > m[j + 6])
                {
                    j += 1;
                    continue;
                }

                // Check levels between spikes
                let high = (m[j] + m[j + 2] + m[j + 7] + m[j + 9]) / 6;
                if m[j + 4] >= high || m[j + 5] >= high {
                    j += 1;
                    continue;
                }

                // Check space between preamble and data
                if m[j + 11] >= high
                    || m[j + 12] >= high
                    || m[j + 13] >= high
                    || m[j + 14] >= high
                {
                    j += 1;
                    continue;
                }
            }

            // Valid preamble found - decode bits
            if use_correction {
                let start = j + MODES_PREAMBLE_US * 2;
                let end = start + aux.len();
                if end <= mlen {
                    aux.copy_from_slice(&magnitude[start..end]);

                    // Apply phase correction if needed
                    if j > 0 {
                        self.apply_phase_correction(&mut aux);
                    }
                }
            }

            // Decode bits
            let (bits, errors) = self.decode_bits(magnitude, j, use_correction, &aux);

            // Pack bits into bytes
            let mut msg = [0u8; 14];
            for i in (0.. MODES_LONG_MSG_BITS).step_by(8) {
                msg[i / 8] = (bits[i] << 7)
                    | (bits[i + 1] << 6)
                    | (bits[i + 2] << 5)
                    | (bits[i + 3] << 4)
                    | (bits[i + 4] << 3)
                    | (bits[i + 5] << 2)
                    | (bits[i + 6] << 1)
                    | bits[i + 7];
            }

            let msg_type = msg[0] >> 3;
            let msg_len = decoder::message_len_by_type(msg_type) / 8;

            // Check signal quality
            let delta = self.compute_signal_delta(magnitude, j, msg_len);
            if delta < 10 * 255 {
                use_correction = false;
                j += 1;
                continue;
            }

            // Decode if error count is acceptable
            if errors == 0 || (self.config.aggressive && errors < 3) {
                let mut mm = decoder::decode_modes_message(
                    &msg[..msg_len],
                    self.config.fix_errors,
                    self.config. aggressive,
                );

                if mm.crc_ok {
                    mm.phase_corrected = use_correction;

                    // Skip past this message
                    j += (MODES_PREAMBLE_US + msg_len * 8) * 2;
                    use_correction = false;

                    // Send to processor
                    let _ = tx.send(mm);
                    continue;
                }
            }

            // Retry with phase correction if first attempt failed
            if !use_correction {
                use_correction = true;
            } else {
                use_correction = false;
                j += 1;
            }
        }
    }

    /// Decode bits from magnitude data. 
    fn decode_bits(
        &self,
        magnitude: &[u16],
        offset: usize,
        use_correction: bool,
        aux:  &[u16],
    ) -> ([u8; MODES_LONG_MSG_BITS], usize) {
        let mut bits = [0u8; MODES_LONG_MSG_BITS];
        let mut errors = 0;

        let preamble_samples = MODES_PREAMBLE_US * 2;

        for i in (0..MODES_LONG_MSG_BITS * 2).step_by(2) {
            let (low, high) = if use_correction && i < aux.len() - 1 {
                (aux[i], aux[i + 1])
            } else {
                let idx = offset + i + preamble_samples;
                if idx + 1 >= magnitude.len() {
                    break;
                }
                (magnitude[idx], magnitude[idx + 1])
            };

            let delta = (low as i32 - high as i32).abs();
            let bit_idx = i / 2;

            if i > 0 && delta < 256 {
                bits[bit_idx] = bits[bit_idx - 1];
            } else if low == high {
                bits[bit_idx] = 2;
                if i < MODES_SHORT_MSG_BITS * 2 {
                    errors += 1;
                }
            } else if low > high {
                bits[bit_idx] = 1;
            } else {
                bits[bit_idx] = 0;
            }
        }

        // Clean up error markers
        for bit in &mut bits {
            if *bit == 2 {
                *bit = 0;
            }
        }

        (bits, errors)
    }

    /// Apply phase correction to magnitude samples. 
    fn apply_phase_correction(&self, m: &mut [u16]) {
        for j in (0..m.len().saturating_sub(2)).step_by(2) {
            if m[j] > m[j + 1] {
                m[j + 2] = (m[j + 2] as u32 * 5 / 4) as u16;
            } else {
                m[j + 2] = (m[j + 2] as u32 * 4 / 5) as u16;
            }
        }
    }

    /// Compute average signal delta for quality assessment
    fn compute_signal_delta(&self, magnitude: &[u16], offset: usize, msg_len: usize) -> u32 {
        let preamble_samples = MODES_PREAMBLE_US * 2;
        let mut delta:  u32 = 0;

        for i in (0..msg_len * 8 * 2).step_by(2) {
            let idx = offset + i + preamble_samples;
            if idx + 1 >= magnitude.len() {
                break;
            }
            delta += (magnitude[idx] as i32 - magnitude[idx + 1] as i32).unsigned_abs();
        }

        delta / (msg_len * 4) as u32
    }
}