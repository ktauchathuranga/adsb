//! I/Q to Magnitude conversion
//!
//! Converts raw 8-bit I/Q samples to magnitude values using a lookup table.

/// Lookup table for I/Q to magnitude conversion.
/// Index: i * 129 + q (where i, q are 0..=128)
/// Value: sqrt(i² + q²) * 360 (scaled to u16 range)
pub struct MagnitudeLut {
    table: Box<[u16; 129 * 129]>,
}

impl MagnitudeLut {
    /// Create a new magnitude lookup table.
    ///
    /// # Algorithm
    /// Raw I/Q values are 0-255, centered at 127.
    /// We compute |I-127| and |Q-127| to get 0-128 range.
    /// Magnitude = sqrt(i² + q²)
    /// Maximum magnitude: sqrt(128² + 128²) ≈ 181.02
    /// Scaled by 360 to use full u16 range.
    pub fn new() -> Self {
        let mut table = vec![0u16; 129 * 129].into_boxed_slice();

        for i in 0..=128u32 {
            for q in 0..=128u32 {
                let mag = ((i * i + q * q) as f64).sqrt() * 360.0;
                table[(i * 129 + q) as usize] = mag.round() as u16;
            }
        }

        // Convert Vec to fixed-size boxed array
        let table: Box<[u16; 129 * 129]> = table.try_into().unwrap();

        Self { table }
    }

    /// Look up magnitude for given I/Q values (already offset-corrected).
    #[inline]
    pub fn lookup(&self, i: u8, q: u8) -> u16 {
        self.table[i as usize * 129 + q as usize]
    }
}

impl Default for MagnitudeLut {
    fn default() -> Self {
        Self::new()
    }
}

/// Convert raw I/Q samples to magnitude vector.
///
/// # Arguments
/// * `data` - Raw I/Q samples (interleaved: I0, Q0, I1, Q1, ...)
/// * `lut` - Magnitude lookup table
///
/// # Returns
/// Vector of magnitude values (one per I/Q pair)
///
/// # C Pointer Arithmetic Conversion
/// Original C:
/// ```c
/// for (j = 0; j < Modes.data_len; j += 2) {
///     int i = p[j]-127;
///     int q = p[j+1]-127;
///     if (i < 0) i = -i;
///     if (q < 0) q = -q;
///     m[j/2] = Modes.maglut[i*129+q];
/// }
/// ```
///
/// Rust version uses:
/// - `chunks_exact(2)` for safe iteration over pairs
/// - `abs_diff` (or explicit abs) for unsigned subtraction
/// - Safe indexing into the LUT
pub fn compute_magnitude_vector(data: &[u8], lut: &MagnitudeLut) -> Vec<u16> {
    let mut magnitude = Vec::with_capacity(data.len() / 2);

    // Process I/Q pairs using safe iteration
    for chunk in data.chunks_exact(2) {
        // Convert from 0-255 range centered at 127 to absolute offset
        let i = (chunk[0] as i16 - 127).unsigned_abs() as u8;
        let q = (chunk[1] as i16 - 127).unsigned_abs() as u8;

        // Clamp to valid LUT range (0-128)
        let i = i.min(128);
        let q = q.min(128);

        magnitude.push(lut.lookup(i, q));
    }

    magnitude
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_magnitude_lut() {
        let lut = MagnitudeLut::new();

        // Zero magnitude at center
        assert_eq!(lut.lookup(0, 0), 0);

        // Maximum magnitude
        let max_mag = lut.lookup(128, 128);
        // sqrt(128² + 128²) * 360 ≈ 65175
        assert!(max_mag > 65000);

        // Single axis
        let single = lut.lookup(128, 0);
        // sqrt(128²) * 360 = 128 * 360 = 46080
        assert!((single as i32 - 46080).abs() < 10);
    }

    #[test]
    fn test_compute_magnitude() {
        let lut = MagnitudeLut::new();

        // Samples at center (127) should give low magnitude
        let data = vec![127u8, 127, 127, 127];
        let mag = compute_magnitude_vector(&data, &lut);
        assert_eq!(mag.len(), 2);
        assert!(mag[0] < 100);

        // Samples at extremes should give high magnitude
        let data = vec![255u8, 255, 0, 0];
        let mag = compute_magnitude_vector(&data, &lut);
        assert!(mag[0] > 40000);
    }
}