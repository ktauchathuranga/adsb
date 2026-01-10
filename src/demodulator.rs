//! Mode S signal demodulation
//!
//!  Detects Mode S preambles and demodulates bit streams from magnitude data.

use std::collections::HashSet;
use std::fs::File;
use std::io::{BufReader, Read};

use crossbeam_channel::Sender;
use tracing::debug;

use crate::config::Config;
use crate::decoder::{self, MODES_LONG_MSG_BITS, ModesMessage};
use crate::magnitude::{MagnitudeLut, compute_magnitude_vector};

/// Preamble duration in microseconds
const MODES_PREAMBLE_US: usize = 8;
/// Full message length for buffer sizing
const MODES_FULL_LEN: usize = MODES_PREAMBLE_US + MODES_LONG_MSG_BITS;
/// Default data buffer length
const MODES_DATA_LEN: usize = 16 * 16384; // 256K

/// Mode S demodulator
pub struct Demodulator {
    config: Config,
    pub mag_lut: MagnitudeLut,
    /// Set of known ICAO addresses (from DF11/DF17 messages)
    known_icaos: HashSet<u32>,
}

impl Demodulator {
    pub fn new(config: Config) -> Self {
        Self {
            config,
            mag_lut: MagnitudeLut::new(),
            known_icaos: HashSet::new(),
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

        let buffer_len = MODES_DATA_LEN + (MODES_FULL_LEN - 1) * 4;
        let mut data = vec![127u8; buffer_len];

        // Track known ICAOs locally for this processing run
        let mut known_icaos: HashSet<u32> = HashSet::new();

        loop {
            let overlap = (MODES_FULL_LEN - 1) * 4;
            data.copy_within(MODES_DATA_LEN..MODES_DATA_LEN + overlap, 0);

            let bytes_read = reader.read(&mut data[overlap..overlap + MODES_DATA_LEN])?;

            if bytes_read == 0 {
                if self.config.loop_file && filename != "-" {
                    drop(reader);
                    if let Ok(file) = File::open(filename) {
                        reader = BufReader::with_capacity(
                            MODES_DATA_LEN,
                            Box::new(file) as Box<dyn Read>,
                        );
                        debug!("Looping file");
                        continue;
                    }
                }
                break;
            }

            if bytes_read < MODES_DATA_LEN {
                data[overlap + bytes_read..].fill(127);
            }

            let magnitude = compute_magnitude_vector(&data[..overlap + bytes_read], &self.mag_lut);
            self.detect_modes_with_icao_tracking(&magnitude, tx, &mut known_icaos);
        }

        Ok(())
    }

    /// Public method for external magnitude data processing
    pub fn detect_modes_external(&self, magnitude: &[u16], tx: &Sender<ModesMessage>) {
        let mut known_icaos: HashSet<u32> = HashSet::new();
        self.detect_modes_with_icao_tracking(magnitude, tx, &mut known_icaos);
    }

    /// Detect Mode S messages in magnitude data with ICAO tracking
    fn detect_modes_with_icao_tracking(
        &self,
        m: &[u16],
        tx: &Sender<ModesMessage>,
        known_icaos: &mut HashSet<u32>,
    ) {
        let mlen = m.len();
        if mlen < MODES_FULL_LEN * 2 {
            return;
        }

        let mut j = 0;

        while j < mlen.saturating_sub(MODES_FULL_LEN * 2) {
            // Check preamble pattern
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

            // Compute high threshold
            let high =
                ((m[j] as u32 + m[j + 2] as u32 + m[j + 7] as u32 + m[j + 9] as u32) / 6) as u16;

            // Check levels between spikes
            if m[j + 4] >= high || m[j + 5] >= high {
                j += 1;
                continue;
            }

            // Check space between preamble and data
            if m[j + 11] >= high || m[j + 12] >= high || m[j + 13] >= high || m[j + 14] >= high {
                j += 1;
                continue;
            }

            // Decode all 112 bits
            let mut bits = [0u8; MODES_LONG_MSG_BITS];
            let preamble_samples = MODES_PREAMBLE_US * 2;

            for i in 0..MODES_LONG_MSG_BITS {
                let idx = j + preamble_samples + i * 2;
                if idx + 1 >= mlen {
                    break;
                }

                let first = m[idx];
                let second = m[idx + 1];

                if first > second {
                    bits[i] = 1;
                } else if first < second {
                    bits[i] = 0;
                } else {
                    bits[i] = if i > 0 { bits[i - 1] } else { 0 };
                }
            }

            // Pack bits into bytes
            let mut msg = [0u8; 14];
            for i in 0..14 {
                msg[i] = (bits[i * 8] << 7)
                    | (bits[i * 8 + 1] << 6)
                    | (bits[i * 8 + 2] << 5)
                    | (bits[i * 8 + 3] << 4)
                    | (bits[i * 8 + 4] << 3)
                    | (bits[i * 8 + 5] << 2)
                    | (bits[i * 8 + 6] << 1)
                    | bits[i * 8 + 7];
            }

            let msg_type = msg[0] >> 3;
            let msg_bits = decoder::message_len_by_type(msg_type);
            let msg_len = msg_bits / 8;

            // Decode the message
            let mut mm = decoder::decode_modes_message(
                &msg[..msg_len],
                self.config.fix_errors,
                self.config.aggressive,
            );

            // For messages with ICAO in CRC, validate against known ICAOs
            let icao_in_message = matches!(mm.msg_type, 11 | 17 | 18);

            if mm.crc_ok && icao_in_message {
                // Valid message with explicit ICAO - add to known set
                known_icaos.insert(mm.icao_address());
                j += (MODES_PREAMBLE_US + msg_len * 8) * 2;
                let _ = tx.send(mm);
            } else if !icao_in_message {
                // DF0, DF4, DF5, DF16, DF20, DF21 - check if recovered ICAO is known
                let recovered_icao = mm.icao_address();
                if known_icaos.contains(&recovered_icao) {
                    mm.crc_ok = true;
                    j += (MODES_PREAMBLE_US + msg_len * 8) * 2;
                    let _ = tx.send(mm);
                } else {
                    j += 1;
                }
            } else {
                j += 1;
            }
        }
    }
}
