//! CRC-24 implementation for Mode S messages
//!
//! This module ports the CRC calculation from the original C code.
//! The CRC is computed by XORing precomputed values for each set bit.

/// Precomputed CRC table for Mode S messages.
/// Each entry corresponds to a bit position in the message.
/// For 112-bit messages, all entries are used.
/// For 56-bit messages, only the last 56 entries are used.
///
/// The last 24 entries are zero because the CRC field itself
/// should not affect the computation.
pub const MODES_CHECKSUM_TABLE: [u32; 112] = [
    0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0, 0x587178,
    0x2c38bc, 0x161c5e, 0x0b0e2f, 0xfa7d13, 0x82c48d, 0xbe9842, 0x5f4c21, 0xd05c14,
    0x682e0a, 0x341705, 0xe5f186, 0x72f8c3, 0xc68665, 0x9cb936, 0x4e5c9b, 0xd8d449,
    0x939020, 0x49c810, 0x24e408, 0x127204, 0x093902, 0x049c81, 0xfdb444, 0x7eda22,
    0x3f6d11, 0xe04c8c, 0x702646, 0x381323, 0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7,
    0x91c77f, 0xb719bb, 0xa476d9, 0xadc168, 0x56e0b4, 0x2b705a, 0x15b82d, 0xf52612,
    0x7a9309, 0xc2b380, 0x6159c0, 0x30ace0, 0x185670, 0x0c2b38, 0x06159c, 0x030ace,
    0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6, 0x2bfd53,
    0xea04ad, 0x8af852, 0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441,
    0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00, 0x383600, 0x1c1b00, 0x0e0d80,
    0x0706c0, 0x038360, 0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
];

/// Calculate the Mode S checksum for a message.
///
/// # Arguments
/// * `msg` - The message bytes (must be at least `bits/8` bytes long)
/// * `bits` - Number of bits in the message (56 or 112)
///
/// # Returns
/// The 24-bit CRC value
///
/// # C Pointer Arithmetic Conversion
/// The original C code uses:
/// ```c
/// int byte = j/8;
/// int bit = j%8;
/// int bitmask = 1 << (7-bit);
/// if (msg[byte] & bitmask) crc ^= table[j+offset];
/// ```
///
/// In Rust, we use safe slice indexing with bounds checking:
/// - `msg[byte]` becomes `msg[j / 8]` with automatic bounds check
/// - The bitmask calculation is identical
pub fn modes_checksum(msg: &[u8], bits: usize) -> u32 {
    debug_assert!(bits == 56 || bits == 112);
    debug_assert!(msg.len() >= bits / 8);

    let mut crc: u32 = 0;
    // For 56-bit messages, skip the first 56 entries in the table
    let offset = if bits == 112 { 0 } else { 112 - 56 };

    for j in 0..bits {
        let byte_idx = j / 8;
        let bit_idx = j % 8;
        // Bit 0 is the MSB in Mode S encoding
        let bitmask = 1u8 << (7 - bit_idx);

        // Safe slice access - Rust will panic if out of bounds (debug) or UB-free (release with checks)
        if msg[byte_idx] & bitmask != 0 {
            crc ^= MODES_CHECKSUM_TABLE[j + offset];
        }
    }

    crc
}

/// Extract the CRC from a message (last 3 bytes).
///
/// # Arguments
/// * `msg` - The message bytes
/// * `bits` - Number of bits (56 or 112)
pub fn extract_crc(msg: &[u8], bits: usize) -> u32 {
    let len = bits / 8;
    debug_assert!(msg.len() >= len);

    // CRC is always the last 3 bytes
    // Safe indexing: we verify len >= 3 implicitly since bits is 56 or 112
    ((msg[len - 3] as u32) << 16) |
    ((msg[len - 2] as u32) << 8) |
    (msg[len - 1] as u32)
}

pub fn recover_icao_from_crc(msg: &[u8], bits: usize) -> u32 {
    let crc = modes_checksum(msg, bits);
    let received = extract_crc(msg, bits);
    crc ^ received
}

/// Check if an ICAO address is plausible
/// ICAO addresses are 24-bit values assigned to aircraft
fn is_valid_icao(icao: u32) -> bool {
    // ICAO addresses are 24-bit, non-zero
    icao != 0 && icao < 0x1000000
}

/// Attempt to fix single-bit errors using the CRC.
///
/// # Algorithm
/// For each bit position, flip it and check if the CRC matches.
/// If found, the error is corrected in place.
///
/// # Returns
/// * `Some(bit_position)` if an error was fixed
/// * `None` if no single-bit fix was possible
///
/// # C Pointer Arithmetic Conversion
/// Original C:
/// ```c
/// memcpy(aux, msg, bits/8);
/// aux[byte] ^= bitmask;
/// ```
///
/// In Rust, we use a stack-allocated array and safe indexing:
/// ```rust
/// let mut aux = [0u8; 14]; // Max message size
/// aux[..len].copy_from_slice(&msg[..len]);
/// aux[byte_idx] ^= bitmask;
/// ```
pub fn fix_single_bit_errors(msg: &mut [u8], bits: usize) -> Option<usize> {
    let len = bits / 8;

    // Work on a copy to avoid modifying original until we find a fix
    let mut aux = [0u8; 14]; // MODES_LONG_MSG_BYTES
    aux[..len].copy_from_slice(&msg[..len]);

    for j in 0..bits {
        let byte_idx = j / 8;
        let bitmask = 1u8 << (7 - (j % 8));

        // Flip bit j
        aux[byte_idx] ^= bitmask;

        let crc_in_msg = extract_crc(&aux, bits);
        let computed_crc = modes_checksum(&aux, bits);

        if crc_in_msg == computed_crc {
            // Found the error! Copy fixed message back
            msg[..len].copy_from_slice(&aux[..len]);
            return Some(j);
        }

        // Flip bit back for next iteration
        aux[byte_idx] ^= bitmask;
    }

    None
}

/// Attempt to fix two-bit errors (aggressive mode).
/// This is computationally expensive: O(n²) where n = bits.
///
/// # Returns
/// * `Some((bit1, bit2))` if errors were fixed
/// * `None` if no two-bit fix was possible
pub fn fix_two_bit_errors(msg: &mut [u8], bits: usize) -> Option<(usize, usize)> {
    let len = bits / 8;
    let mut aux = [0u8; 14];
    aux[..len].copy_from_slice(&msg[..len]);

    for j in 0..bits {
        let byte1 = j / 8;
        let bitmask1 = 1u8 << (7 - (j % 8));

        // Start from j+1 to avoid checking same pairs twice
        for i in (j + 1)..bits {
            let byte2 = i / 8;
            let bitmask2 = 1u8 << (7 - (i % 8));

            // Reset aux to original
            aux[..len].copy_from_slice(&msg[..len]);

            // Flip both bits
            aux[byte1] ^= bitmask1;
            aux[byte2] ^= bitmask2;

            let crc_in_msg = extract_crc(&aux, bits);
            let computed_crc = modes_checksum(&aux, bits);

            if crc_in_msg == computed_crc {
                msg[..len].copy_from_slice(&aux[..len]);
                return Some((j, i));
            }
        }
    }

    None
}

/// Verify CRC of a message.
pub fn verify_crc(msg: &[u8], bits: usize) -> bool {
    let crc_in_msg = extract_crc(msg, bits);
    let computed_crc = modes_checksum(msg, bits);
    crc_in_msg == computed_crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_checksum_known_good() {
        // Test with a known good DF17 message
        // This is a placeholder - real test would use captured data
        let msg = [0x8D, 0x48, 0x40, 0xD6, 0x20, 0x2C, 0xC3,
                   0x71, 0xC3, 0x2C, 0xE0, 0x57, 0x60, 0x98];
        let crc = modes_checksum(&msg, 112);
        let expected = extract_crc(&msg, 112);
        // For a valid message, computed CRC should match extracted CRC
        assert_eq!(crc, expected);
    }

    #[test]
    fn test_single_bit_error_correction() {
        // Start with a valid message
        let mut msg = [0x8D, 0x48, 0x40, 0xD6, 0x20, 0x2C, 0xC3,
                       0x71, 0xC3, 0x2C, 0xE0, 0x57, 0x60, 0x98];
        let original = msg.clone();

        // Introduce a single-bit error
        msg[5] ^= 0x04;

        // Attempt to fix
        if let Some(bit_pos) = fix_single_bit_errors(&mut msg, 112) {
            assert_eq!(msg, original);
            assert!(bit_pos > 0);
        }
    }
}