//! Signal Processing Utilities
//!
//! Provides SNR estimation, noise floor tracking, and signal quality metrics.

use std::collections::VecDeque;

/// Number of samples to use for noise floor estimation
const NOISE_FLOOR_SAMPLES: usize = 256;

/// Minimum SNR (in dB) to consider a message reliable
#[allow(dead_code)]
pub const MIN_RELIABLE_SNR_DB: f32 = 3.0;

/// Signal quality metrics for a decoded message
#[allow(dead_code)]
#[derive(Debug, Clone, Default)]
pub struct SignalStats {
    /// Signal-to-noise ratio in dB
    pub snr_db: f32,
    /// Peak signal level (magnitude units)
    pub signal_level: u16,
    /// Estimated noise floor (magnitude units)
    pub noise_level: u16,
    /// Whether phase correction was applied
    pub phase_corrected: bool,
}

impl SignalStats {
    /// Check if signal quality is good enough to trust
    #[allow(dead_code)]
    pub fn is_reliable(&self) -> bool {
        self.snr_db >= MIN_RELIABLE_SNR_DB
    }
}

/// Signal processor with noise floor tracking
pub struct SignalProcessor {
    /// Running estimate of noise floor
    noise_floor: u16,
    /// Recent noise samples for averaging
    noise_samples: VecDeque<u16>,
    /// Adaptive threshold multiplier
    #[allow(dead_code)]
    threshold_multiplier: f32,
}

impl Default for SignalProcessor {
    fn default() -> Self {
        Self::new()
    }
}

impl SignalProcessor {
    /// Create a new signal processor
    pub fn new() -> Self {
        Self {
            noise_floor: 100, // Initial estimate
            noise_samples: VecDeque::with_capacity(NOISE_FLOOR_SAMPLES),
            threshold_multiplier: 2.5,
        }
    }

    /// Get current noise floor estimate
    #[allow(dead_code)]
    pub fn noise_floor(&self) -> u16 {
        self.noise_floor
    }

    /// Get adaptive threshold for preamble detection
    #[allow(dead_code)]
    pub fn adaptive_threshold(&self) -> u16 {
        ((self.noise_floor as f32) * self.threshold_multiplier) as u16
    }

    /// Estimate noise floor from magnitude samples
    /// Uses the lower quartile of samples as noise estimate
    pub fn update_noise_floor(&mut self, magnitude: &[u16]) {
        if magnitude.len() < 100 {
            return;
        }

        // Sample every 16th value to reduce computation
        let step = 16;
        let mut samples: Vec<u16> = magnitude.iter().step_by(step).copied().collect();
        
        if samples.is_empty() {
            return;
        }

        // Sort to find lower quartile (25th percentile)
        samples.sort_unstable();
        let quartile_idx = samples.len() / 4;
        let noise_estimate = samples[quartile_idx];

        // Add to running average
        self.noise_samples.push_back(noise_estimate);
        if self.noise_samples.len() > NOISE_FLOOR_SAMPLES {
            self.noise_samples.pop_front();
        }

        // Update noise floor as average of recent estimates
        if !self.noise_samples.is_empty() {
            let sum: u32 = self.noise_samples.iter().map(|&x| x as u32).sum();
            self.noise_floor = (sum / self.noise_samples.len() as u32) as u16;
        }
    }

    /// Calculate SNR in dB for a given signal level
    pub fn calculate_snr_db(&self, signal_level: u16) -> f32 {
        if self.noise_floor == 0 || signal_level <= self.noise_floor {
            return 0.0;
        }

        let signal = signal_level as f32;
        let noise = self.noise_floor as f32;
        
        // SNR in dB = 20 * log10(signal / noise)
        20.0 * (signal / noise).log10()
    }

    /// Get signal stats for a message based on preamble peaks
    #[allow(dead_code)]
    pub fn get_signal_stats(&self, preamble_peaks: &[u16]) -> SignalStats {
        if preamble_peaks.is_empty() {
            return SignalStats::default();
        }

        // Average the preamble peaks for signal level
        let sum: u32 = preamble_peaks.iter().map(|&x| x as u32).sum();
        let signal_level = (sum / preamble_peaks.len() as u32) as u16;

        SignalStats {
            snr_db: self.calculate_snr_db(signal_level),
            signal_level,
            noise_level: self.noise_floor,
            phase_corrected: false,
        }
    }

    /// Check if a signal level is strong enough to attempt phase correction
    /// (Worth the extra CPU for marginal signals)
    pub fn should_try_phase_correction(&self, signal_level: u16) -> bool {
        let snr = self.calculate_snr_db(signal_level);
        // Try phase correction for signals between 2-8 dB SNR
        // Below 2 dB is too weak, above 8 dB should decode fine
        snr >= 2.0 && snr <= 8.0
    }
}

/// Detect if a message might benefit from phase correction
/// by checking the bit confidence at sampling points
#[allow(dead_code)]
pub fn check_phase_ambiguity(magnitude: &[u16], bit_start: usize, num_bits: usize) -> bool {
    let mut ambiguous_bits = 0;
    
    for i in 0..num_bits.min(56) { // Check first 56 bits
        let idx = bit_start + i * 2;
        if idx + 1 >= magnitude.len() {
            break;
        }

        let first = magnitude[idx] as i32;
        let second = magnitude[idx + 1] as i32;
        let diff = (first - second).abs();
        
        // If samples are very close, the bit decision is ambiguous
        let avg = ((first + second) / 2).max(1);
        if diff * 10 < avg { // Less than 10% difference
            ambiguous_bits += 1;
        }
    }

    // If more than 15% of bits are ambiguous, try phase correction
    ambiguous_bits > (num_bits.min(56) * 15 / 100)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_signal_processor_noise_floor() {
        let mut sp = SignalProcessor::new();
        
        // Simulate low noise samples
        let magnitude: Vec<u16> = (0..1000).map(|i| 50 + (i % 20) as u16).collect();
        sp.update_noise_floor(&magnitude);
        
        // Noise floor should be around 50-60
        assert!(sp.noise_floor() >= 40 && sp.noise_floor() <= 80);
    }

    #[test]
    fn test_snr_calculation() {
        let mut sp = SignalProcessor::new();
        sp.noise_floor = 100;

        // Signal at 1000 should give ~20 dB SNR
        let snr = sp.calculate_snr_db(1000);
        assert!(snr >= 19.0 && snr <= 21.0);

        // Signal at noise floor should give 0 dB
        let snr = sp.calculate_snr_db(100);
        assert!(snr < 0.1);
    }

    #[test]
    fn test_adaptive_threshold() {
        let mut sp = SignalProcessor::new();
        sp.noise_floor = 100;
        
        // Threshold should be noise_floor * multiplier
        let threshold = sp.adaptive_threshold();
        assert!(threshold >= 200 && threshold <= 300);
    }
}
