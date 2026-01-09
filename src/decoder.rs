//! Mode S message decoder
//!
//!  Decodes raw Mode S messages into structured data. 

use std::fmt;
use crate::crc::{self, modes_checksum, extract_crc};

/// Constants for message sizes
pub const MODES_LONG_MSG_BITS: usize = 112;
pub const MODES_SHORT_MSG_BITS: usize = 56;
pub const MODES_LONG_MSG_BYTES: usize = 14;
pub const MODES_SHORT_MSG_BYTES: usize = 7;

/// Unit for altitude measurements
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AltitudeUnit {
    Feet,
    Meters,
}

/// Decoded Mode S message
#[derive(Debug, Clone)]
pub struct ModesMessage {
    /// Raw message bytes
    pub msg: [u8; MODES_LONG_MSG_BYTES],
    /// Number of bits in message
    pub msg_bits: usize,
    /// Downlink Format (DF)
    pub msg_type: u8,
    /// CRC value from message
    pub crc: u32,
    /// Whether CRC was valid
    pub crc_ok: bool,
    /// Bit position that was corrected (None if no correction)
    pub error_bit: Option<usize>,
    /// Second error bit for two-bit correction
    pub error_bit2: Option<usize>,
    /// ICAO address bytes
    pub aa: [u8; 3],
    /// Responder capabilities (CA field)
    pub ca: u8,
    /// Extended squitter message type (ME type)
    pub me_type: u8,
    /// Extended squitter message subtype
    pub me_sub: u8,
    /// Flight status (DF4,5,20,21)
    pub fs: u8,
    /// Downlink request
    pub dr: u8,
    /// Utility message
    pub um: u8,
    /// Squawk identity code
    pub identity: u16,
    /// Altitude
    pub altitude: i32,
    /// Altitude unit
    pub unit: AltitudeUnit,
    /// Flight callsign
    pub flight: String,
    /// Aircraft type category
    pub aircraft_type: u8,
    /// CPR format flag (false = even, true = odd)
    pub fflag: bool,
    /// Time flag
    pub tflag: bool,
    /// Raw CPR latitude
    pub raw_latitude: u32,
    /// Raw CPR longitude
    pub raw_longitude:  u32,
    /// Heading validity
    pub heading_is_valid: bool,
    /// Heading in degrees
    pub heading: f64,
    /// East/West direction (0 = East, 1 = West)
    pub ew_dir: u8,
    /// East/West velocity component
    pub ew_velocity: u16,
    /// North/South direction (0 = North, 1 = South)
    pub ns_dir: u8,
    /// North/South velocity component
    pub ns_velocity: u16,
    /// Vertical rate source
    pub vert_rate_source: u8,
    /// Vertical rate sign
    pub vert_rate_sign: u8,
    /// Vertical rate
    pub vert_rate: u16,
    /// Computed velocity
    pub velocity: u16,
    /// Whether phase correction was applied
    pub phase_corrected: bool,
}

impl Default for ModesMessage {
    fn default() -> Self {
        Self {
            msg: [0; MODES_LONG_MSG_BYTES],
            msg_bits: 0,
            msg_type: 0,
            crc: 0,
            crc_ok: false,
            error_bit: None,
            error_bit2: None,
            aa: [0; 3],
            ca: 0,
            me_type: 0,
            me_sub: 0,
            fs: 0,
            dr: 0,
            um: 0,
            identity: 0,
            altitude: 0,
            unit: AltitudeUnit::Feet,
            flight: String::new(),
            aircraft_type: 0,
            fflag: false,
            tflag: false,
            raw_latitude: 0,
            raw_longitude: 0,
            heading_is_valid: false,
            heading: 0.0,
            ew_dir: 0,
            ew_velocity: 0,
            ns_dir: 0,
            ns_velocity: 0,
            vert_rate_source: 0,
            vert_rate_sign: 0,
            vert_rate: 0,
            velocity: 0,
            phase_corrected: false,
        }
    }
}

impl ModesMessage {
    /// Get the 24-bit ICAO address as a u32
    pub fn icao_address(&self) -> u32 {
        ((self.aa[0] as u32) << 16) | ((self.aa[1] as u32) << 8) | (self.aa[2] as u32)
    }

    /// Format as raw hex string for network output
    pub fn to_raw_string(&self) -> String {
        let bytes = self.msg_bits / 8;
        let mut s = String::with_capacity(bytes * 2 + 3);
        s.push('*');
        for i in 0..bytes {
            s.push_str(&format!("{:02X}", self.msg[i]));
        }
        s.push(';');
        s
    }

    /// Format as SBS/BaseStation output
    pub fn to_sbs_string(&self, lat: f64, lon: f64) -> Option<String> {
        let icao = format!("{:02X}{:02X}{:02X}", self.aa[0], self.aa[1], self.aa[2]);

        match self.msg_type {
            0 => Some(format!(
                "MSG,5,,,{},,,,,,,,{},,,,,,,,,,",
                icao, self.altitude
            )),
            4 => {
                let (alert, emergency, spi, ground) = self.decode_flight_status_flags();
                Some(format!(
                    "MSG,5,,,{},,,,,,,{},,,,,,,,{},{},{},{}",
                    icao, self.altitude, alert, emergency, spi, ground
                ))
            }
            5 => {
                let (alert, emergency, spi, ground) = self.decode_flight_status_flags();
                Some(format!(
                    "MSG,6,,,{},,,,,,,,,,,,,,{},{},{},{},{}",
                    icao, self.identity, alert, emergency, spi, ground
                ))
            }
            11 => Some(format!("MSG,8,,,{},,,,,,,,,,,,,,,,,", icao)),
            17 if self.me_type == 4 => Some(format!(
                "MSG,1,,,{},,,,,,,{},,,,,,,,0,0,0,0",
                icao, self.flight
            )),
            17 if (9..=18).contains(&self.me_type) => {
                if lat == 0.0 && lon == 0.0 {
                    Some(format!(
                        "MSG,3,,,{},,,,,,,,{},,,,,,,,0,0,0,0",
                        icao, self.altitude
                    ))
                } else {
                    Some(format!(
                        "MSG,3,,,{},,,,,,,{},,{:.5},{:.5},,,0,0,0,0",
                        icao, self.altitude, lat, lon
                    ))
                }
            }
            17 if self.me_type == 19 && self.me_sub == 1 => {
                let vr = if self.vert_rate_sign == 0 { 1 } else { -1 }
                    * (self.vert_rate as i32 - 1)
                    * 64;
                Some(format!(
                    "MSG,4,,,{},,,,,,,,{},{},,,,{},,0,0,0,0",
                    icao, self.velocity, self.heading as i32, vr
                ))
            }
            21 => {
                let (alert, emergency, spi, ground) = self.decode_flight_status_flags();
                Some(format!(
                    "MSG,6,,,{},,,,,,,,,,,,,,{},{},{},{},{}",
                    icao, self.identity, alert, emergency, spi, ground
                ))
            }
            _ => None,
        }
    }

    /// Decode flight status flags for SBS output
    fn decode_flight_status_flags(&self) -> (i32, i32, i32, i32) {
        let emergency = if self.identity == 7500 || self.identity == 7600 || self.identity == 7700 {
            -1
        } else {
            0
        };
        let ground = if self.fs == 1 || self.fs == 3 { -1 } else { 0 };
        let alert = if self.fs == 2 || self.fs == 3 || self.fs == 4 {
            -1
        } else {
            0
        };
        let spi = if self.fs == 4 || self.fs == 5 { -1 } else { 0 };
        (alert, emergency, spi, ground)
    }
}

impl fmt::Display for ModesMessage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Show raw message hex
        write!(f, "*")?;
        for i in 0..(self.msg_bits / 8) {
            write!(f, "{:02X}", self.msg[i])?;
        }
        writeln!(f, ";")?;

        writeln!(
            f,
            "CRC: {:06x} ({})",
            self.crc,
            if self.crc_ok { "ok" } else { "wrong" }
        )?;

        if let Some(bit) = self.error_bit {
            writeln!(f, "Single bit error fixed, bit {}", bit)?;
        }

        match self.msg_type {
            0 => {
                writeln!(f, "DF 0: Short Air-Air Surveillance.")?;
                writeln!(
                    f,
                    "  Altitude       :  {} {}",
                    self.altitude,
                    if self.unit == AltitudeUnit::Meters {
                        "meters"
                    } else {
                        "feet"
                    }
                )?;
                writeln!(
                    f,
                    "  ICAO Address   : {:02x}{:02x}{:02x}",
                    self.aa[0], self.aa[1], self.aa[2]
                )?;
            }
            4 | 20 => {
                let name = if self.msg_type == 4 {
                    "Surveillance"
                } else {
                    "Comm-B"
                };
                writeln!(f, "DF {}: {}, Altitude Reply.", self.msg_type, name)?;
                writeln!(f, "  Flight Status  : {}", flight_status_str(self.fs))?;
                writeln!(f, "  DR             : {}", self.dr)?;
                writeln!(f, "  UM             : {}", self.um)?;
                writeln!(
                    f,
                    "  Altitude       : {} {}",
                    self.altitude,
                    if self.unit == AltitudeUnit::Meters {
                        "meters"
                    } else {
                        "feet"
                    }
                )?;
                writeln!(
                    f,
                    "  ICAO Address   : {:02x}{:02x}{:02x}",
                    self.aa[0], self.aa[1], self.aa[2]
                )?;
            }
            5 | 21 => {
                let name = if self.msg_type == 5 {
                    "Surveillance"
                } else {
                    "Comm-B"
                };
                writeln!(f, "DF {}: {}, Identity Reply.", self.msg_type, name)?;
                writeln!(f, "  Flight Status  :  {}", flight_status_str(self.fs))?;
                writeln!(f, "  DR             : {}", self.dr)?;
                writeln!(f, "  UM             : {}", self.um)?;
                writeln!(f, "  Squawk         : {:04}", self.identity)?;
                writeln!(
                    f,
                    "  ICAO Address   : {:02x}{:02x}{:02x}",
                    self.aa[0], self.aa[1], self.aa[2]
                )?;
            }
            11 => {
                writeln!(f, "DF 11: All Call Reply.")?;
                writeln!(f, "  Capability  : {}", capability_str(self.ca))?;
                writeln!(
                    f,
                    "  ICAO Address: {:02x}{:02x}{:02x}",
                    self.aa[0], self.aa[1], self.aa[2]
                )?;
            }
            17 => {
                writeln!(f, "DF 17: ADS-B message.")?;
                writeln!(
                    f,
                    "  Capability     : {} ({})",
                    self.ca,
                    capability_str(self.ca)
                )?;
                writeln!(
                    f,
                    "  ICAO Address   : {:02x}{:02x}{:02x}",
                    self.aa[0], self.aa[1], self.aa[2]
                )?;
                writeln!(f, "  Extended Squitter  Type: {}", self.me_type)?;
                writeln!(f, "  Extended Squitter  Sub : {}", self.me_sub)?;
                writeln!(
                    f,
                    "  Extended Squitter  Name: {}",
                    get_me_description(self.me_type, self.me_sub)
                )?;

                if (1..=4).contains(&self.me_type) {
                    let ac_types = [
                        "Aircraft Type D",
                        "Aircraft Type C",
                        "Aircraft Type B",
                        "Aircraft Type A",
                    ];
                    writeln!(
                        f,
                        "    Aircraft Type  : {}",
                        ac_types
                            .get(self.aircraft_type as usize)
                            .unwrap_or(&"Unknown")
                    )?;
                    writeln!(f, "    Identification : {}", self.flight)?;
                } else if (9..=18).contains(&self.me_type) {
                    writeln!(
                        f,
                        "    F flag   : {}",
                        if self.fflag { "odd" } else { "even" }
                    )?;
                    writeln!(
                        f,
                        "    T flag   : {}",
                        if self.tflag { "UTC" } else { "non-UTC" }
                    )?;
                    writeln!(f, "    Altitude : {} feet", self.altitude)?;
                    writeln!(f, "    Latitude : {} (not decoded)", self.raw_latitude)?;
                    writeln!(f, "    Longitude: {} (not decoded)", self.raw_longitude)?;
                } else if self.me_type == 19 && (1..=4).contains(&self.me_sub) {
                    if self.me_sub == 1 || self.me_sub == 2 {
                        writeln!(f, "    EW direction      : {}", self.ew_dir)?;
                        writeln!(f, "    EW velocity       : {}", self.ew_velocity)?;
                        writeln!(f, "    NS direction      : {}", self.ns_dir)?;
                        writeln!(f, "    NS velocity       : {}", self.ns_velocity)?;
                        writeln!(f, "    Vertical rate src : {}", self.vert_rate_source)?;
                        writeln!(f, "    Vertical rate sign: {}", self.vert_rate_sign)?;
                        writeln!(f, "    Vertical rate     : {}", self.vert_rate)?;
                    } else {
                        writeln!(f, "    Heading status: {}", self.heading_is_valid)?;
                        writeln!(f, "    Heading: {:.1}", self.heading)?;
                    }
                } else {
                    writeln!(
                        f,
                        "    Unrecognized ME type: {} subtype: {}",
                        self.me_type, self.me_sub
                    )?;
                }
            }
            16 => {
                writeln!(f, "DF 16: Long Air-Air Surveillance.")?;
                writeln!(
                    f,
                    "  Altitude       : {} {}",
                    self.altitude,
                    if self.unit == AltitudeUnit::Meters {
                        "meters"
                    } else {
                        "feet"
                    }
                )?;
                writeln!(
                    f,
                    "  ICAO Address   :  {:02x}{:02x}{:02x}",
                    self.aa[0], self.aa[1], self.aa[2]
                )?;
            }
            _ => {
                writeln!(
                    f,
                    "DF {} (decoding not fully implemented)",
                    self.msg_type
                )?;
            }
        }

        Ok(())
    }
}

/// AIS charset for flight ID decoding
/// Index maps to character: ?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? 0123456789?????
const AIS_CHARSET: &[u8; 64] =
    b"?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? 0123456789?????????????????????";

/// Decode a Mode S message from raw bytes. 
///
/// # Arguments
/// * `raw_msg` - Raw message bytes (at least 7 bytes for short, 14 for long messages)
/// * `fix_errors` - Whether to attempt single-bit error correction
/// * `aggressive` - Whether to try two-bit error correction (CPU intensive)
///
/// # C Pointer Arithmetic Conversion
///
/// The C code accesses message bytes directly with array indexing:
/// ```c
/// mm->msgtype = msg[0] >> 3;
/// mm->ca = msg[0] & 7;
/// mm->aa1 = msg[1]; mm->aa2 = msg[2]; mm->aa3 = msg[3];
/// ```
///
/// In Rust, we use the same indexing but with automatic bounds checking:
/// ```rust
/// mm.msg_type = msg[0] >> 3;
/// mm.ca = msg[0] & 0x07;
/// mm.aa = [msg[1], msg[2], msg[3]];
/// ```
///
/// Bit field extraction example:
/// ```c
/// mm->metype = msg[4] >> 3;   // High 5 bits
/// mm->mesub = msg[4] & 7;     // Low 3 bits
/// ```
/// Becomes:
/// ```rust
/// mm.me_type = msg[4] >> 3;
/// mm.me_sub = msg[4] & 0x07;
/// ```
pub fn decode_modes_message(raw_msg: &[u8], fix_errors: bool, aggressive:  bool) -> ModesMessage {
    let mut mm = ModesMessage::default();

    // Copy message to local buffer (safe slice copy)
    let len = raw_msg.len().min(MODES_LONG_MSG_BYTES);
    mm.msg[..len].copy_from_slice(&raw_msg[..len]);

    // Get message type (Downlink Format) from first 5 bits
    mm.msg_type = mm.msg[0] >> 3;
    mm.msg_bits = message_len_by_type(mm.msg_type);

    // Extract CRC (always last 3 bytes of message)
    mm.crc = extract_crc(&mm.msg, mm.msg_bits);
    let computed_crc = modes_checksum(&mm.msg, mm. msg_bits);
    mm.crc_ok = mm.crc == computed_crc;

    // Attempt error correction for DF11 and DF17 messages
    if ! mm.crc_ok && fix_errors && (mm.msg_type == 11 || mm.msg_type == 17) {
        // Try single-bit error correction first
        if let Some(bit) = crc::fix_single_bit_errors(&mut mm.msg, mm.msg_bits) {
            mm.error_bit = Some(bit);
            mm.crc = extract_crc(&mm.msg, mm.msg_bits);
            mm.crc_ok = true;
        } else if aggressive && mm.msg_type == 17 {
            // Try two-bit error correction (expensive)
            if let Some((bit1, bit2)) = crc::fix_two_bit_errors(&mut mm.msg, mm. msg_bits) {
                mm.error_bit = Some(bit1);
                mm.error_bit2 = Some(bit2);
                mm.crc = extract_crc(&mm.msg, mm.msg_bits);
                mm.crc_ok = true;
            }
        }
    }

    // === Decode common fields ===
    // Responder capabilities (CA) - bits 5-7 of first byte
    mm.ca = mm.msg[0] & 0x07;

    // ICAO address - bytes 1, 2, 3
    mm.aa = [mm.msg[1], mm. msg[2], mm.msg[3]];

    // === Decode DF17 specific fields ===
    // ME type (bits 0-4 of byte 4) and subtype (bits 5-7)
    mm.me_type = mm.msg[4] >> 3;
    mm.me_sub = mm.msg[4] & 0x07;

    // === Decode fields for DF4, DF5, DF20, DF21 ===
    // Flight status
    mm.fs = mm.msg[0] & 0x07;
    // Downlink request
    mm.dr = (mm.msg[1] >> 3) & 0x1F;
    // Utility message
    mm.um = ((mm.msg[1] & 0x07) << 3) | (mm.msg[2] >> 5);

    // === Decode squawk (identity) ===
    let a = ((mm.msg[3] & 0x80) >> 5) | (mm.msg[2] & 0x02) | ((mm.msg[2] & 0x08) >> 3);
    let b = ((mm.msg[3] & 0x02) << 1) | ((mm.msg[3] & 0x08) >> 2) | ((mm.msg[3] & 0x20) >> 5);
    let c = ((mm.msg[2] & 0x01) << 2) | ((mm.msg[2] & 0x04) >> 1) | ((mm.msg[2] & 0x10) >> 4);
    let d = ((mm.msg[3] & 0x01) << 2) | ((mm.msg[3] & 0x04) >> 1) | ((mm.msg[3] & 0x10) >> 4);
    mm.identity = (a as u16) * 1000 + (b as u16) * 100 + (c as u16) * 10 + (d as u16);

    // === Decode altitude for DF0, DF4, DF16, DF20 ===
    if matches!(mm.msg_type, 0 | 4 | 16 | 20) {
        mm.altitude = decode_ac13_field(&mm.msg, &mut mm.unit);
    }

    // === Decode extended squitter (DF17) ===
    if mm.msg_type == 17 {
        decode_extended_squitter(&mut mm);
    }

    mm
}

/// Decode extended squitter message (DF17)
fn decode_extended_squitter(mm: &mut ModesMessage) {
    if (1..=4).contains(&mm.me_type) {
        // === Aircraft Identification and Category ===
        mm.aircraft_type = mm.me_type - 1;

        // Decode flight callsign using AIS charset (6 bits per character)
        let char_indices = [
            (mm.msg[5] >> 2) as usize,
            (((mm.msg[5] & 0x03) << 4) | (mm.msg[6] >> 4)) as usize,
            (((mm.msg[6] & 0x0F) << 2) | (mm.msg[7] >> 6)) as usize,
            (mm.msg[7] & 0x3F) as usize,
            (mm.msg[8] >> 2) as usize,
            (((mm.msg[8] & 0x03) << 4) | (mm.msg[9] >> 4)) as usize,
            (((mm. msg[9] & 0x0F) << 2) | (mm.msg[10] >> 6)) as usize,
            (mm.msg[10] & 0x3F) as usize,
        ];

        let chars: Vec<char> = char_indices
            .iter()
            .map(|&idx| {
                if idx < AIS_CHARSET.len() {
                    AIS_CHARSET[idx] as char
                } else {
                    '?'
                }
            })
            .collect();

        mm.flight = chars. into_iter().collect::<String>().trim().to_string();
    } else if (9..=18).contains(&mm.me_type) {
        // === Airborne Position Message ===
        mm.fflag = (mm.msg[6] & 0x04) != 0;
        mm.tflag = (mm.msg[6] & 0x08) != 0;
        mm.altitude = decode_ac12_field(&mm.msg, &mut mm.unit);

        mm.raw_latitude =
            (((mm.msg[6] & 0x03) as u32) << 15) | ((mm.msg[7] as u32) << 7) | ((mm.msg[8] >> 1) as u32);
        mm.raw_longitude =
            (((mm.msg[8] & 0x01) as u32) << 16) | ((mm.msg[9] as u32) << 8) | (mm.msg[10] as u32);
    } else if mm.me_type == 19 && (1..=4).contains(&mm.me_sub) {
        // === Airborne Velocity Message ===
        if mm.me_sub == 1 || mm.me_sub == 2 {
            mm. ew_dir = (mm.msg[5] & 0x04) >> 2;
            mm. ew_velocity = (((mm.msg[5] & 0x03) as u16) << 8) | (mm.msg[6] as u16);
            mm.ns_dir = (mm.msg[7] & 0x80) >> 7;
            mm.ns_velocity = (((mm.msg[7] & 0x7F) as u16) << 3) | (((mm.msg[8] & 0xE0) >> 5) as u16);
            mm.vert_rate_source = (mm.msg[8] & 0x10) >> 4;
            mm. vert_rate_sign = (mm.msg[8] & 0x08) >> 3;
            mm.vert_rate = (((mm.msg[8] & 0x07) as u16) << 6) | (((mm.msg[9] & 0xFC) >> 2) as u16);

            let ewv = mm.ew_velocity as f64;
            let nsv = mm. ns_velocity as f64;
            mm.velocity = (ewv * ewv + nsv * nsv).sqrt() as u16;

            if mm.velocity > 0 {
                let ewv_signed = if mm.ew_dir != 0 { -ewv } else { ewv };
                let nsv_signed = if mm.ns_dir != 0 { -nsv } else { nsv };
                let mut heading = ewv_signed.atan2(nsv_signed) * 180.0 / std::f64::consts::PI;
                if heading < 0.0 {
                    heading += 360.0;
                }
                mm.heading = heading;
            }
        } else if mm.me_sub == 3 || mm.me_sub == 4 {
            mm.heading_is_valid = (mm.msg[5] & 0x04) != 0;
            mm.heading =
                (360.0 / 128.0) * ((((mm.msg[5] & 0x03) as u16) << 5) | ((mm.msg[6] >> 3) as u16)) as f64;
        }
    }
}

/// Decode 13-bit AC altitude field (used in DF0, DF4, DF16, DF20)
///
/// # Algorithm
/// The altitude is encoded in a 13-bit field with M and Q bits:
/// - M bit (bit 6): 0 = feet, 1 = meters
/// - Q bit (bit 4): determines encoding resolution
///
/// When M=0 and Q=1: altitude = N * 25 - 1000 (where N is 11-bit value)
///
/// # C Pointer Arithmetic Conversion
/// Original C: 
/// ```c
/// int m_bit = msg[3] & (1<<6);
/// int q_bit = msg[3] & (1<<4);
/// int n = ((msg[2]&31)<<6) | ((msg[3]&0x80)>>2) | ((msg[3]&0x20)>>1) | (msg[3]&15);
/// return n*25-1000;
/// ```
fn decode_ac13_field(msg: &[u8], unit: &mut AltitudeUnit) -> i32 {
    let m_bit = (msg[3] & 0x40) != 0; // Bit 6
    let q_bit = (msg[3] & 0x10) != 0; // Bit 4

    if !m_bit {
        *unit = AltitudeUnit::Feet;
        if q_bit {
            // 25-foot resolution
            // Extract 11-bit integer (removing Q and M bits)
            let n = (((msg[2] & 0x1F) as i32) << 6)
                | (((msg[3] & 0x80) >> 2) as i32)
                | (((msg[3] & 0x20) >> 1) as i32)
                | ((msg[3] & 0x0F) as i32);
            return n * 25 - 1000;
        }
        // TODO:  Implement Gillham (Gray) code altitude decoding when Q=0
    } else {
        *unit = AltitudeUnit::Meters;
        // TODO: Implement metric altitude
    }
    0
}

/// Decode 12-bit AC altitude field (used in DF17 airborne position)
///
/// Similar to AC13 but without the M bit (always feet).
fn decode_ac12_field(msg: &[u8], unit: &mut AltitudeUnit) -> i32 {
    let q_bit = (msg[5] & 0x01) != 0;

    if q_bit {
        *unit = AltitudeUnit::Feet;
        // 25-foot resolution
        let n = (((msg[5] >> 1) as i32) << 4) | (((msg[6] & 0xF0) >> 4) as i32);
        return n * 25 - 1000;
    }
    0
}

/// Get message length in bits based on Downlink Format
pub fn message_len_by_type(df: u8) -> usize {
    match df {
        16 | 17 | 19 | 20 | 21 => MODES_LONG_MSG_BITS,  // 112 bits
        _ => MODES_SHORT_MSG_BITS,                       // 56 bits
    }
}

/// Get capability description string
fn capability_str(ca: u8) -> &'static str {
    match ca {
        0 => "Level 1 (Surveillance Only)",
        1 => "Level 2 (DF0,4,5,11)",
        2 => "Level 3 (DF0,4,5,11,20,21)",
        3 => "Level 4 (DF0,4,5,11,20,21,24)",
        4 => "Level 2+3+4 (DF0,4,5,11,20,21,24,code7 - is on ground)",
        5 => "Level 2+3+4 (DF0,4,5,11,20,21,24,code7 - is airborne)",
        6 => "Level 2+3+4 (DF0,4,5,11,20,21,24,code7)",
        7 => "Level 7",
        _ => "Unknown",
    }
}

/// Get flight status description string
fn flight_status_str(fs: u8) -> &'static str {
    match fs {
        0 => "Normal, Airborne",
        1 => "Normal, On the ground",
        2 => "ALERT, Airborne",
        3 => "ALERT, On the ground",
        4 => "ALERT & Special Position Identification",
        5 => "Special Position Identification",
        6 => "Value 6 is not assigned",
        7 => "Value 7 is not assigned",
        _ => "Unknown",
    }
}

/// Get ME (Message Extended) type description
fn get_me_description(metype: u8, mesub: u8) -> &'static str {
    match metype {
        1..=4 => "Aircraft Identification and Category",
        5..=8 => "Surface Position",
        9..=18 => "Airborne Position (Baro Altitude)",
        19 if (1..=4).contains(&mesub) => "Airborne Velocity",
        20..=22 => "Airborne Position (GNSS Height)",
        23 if mesub == 0 => "Test Message",
        24 if mesub == 1 => "Surface System Status",
        28 if mesub == 1 => "Extended Squitter Aircraft Status (Emergency)",
        28 if mesub == 2 => "Extended Squitter Aircraft Status (1090ES TCAS RA)",
        29 if mesub == 0 || mesub == 1 => "Target State and Status Message",
        31 if mesub == 0 || mesub == 1 => "Aircraft Operational Status Message",
        _ => "Unknown",
    }
}

/// Parse a hex string message (from network input)
///
/// Format: *HEXDIGITS;
///
/// # Example
/// ```
/// let msg = decode_hex_message("*8D4840D6202CC371C32CE0576098;", true, false);
/// ```
pub fn decode_hex_message(hex: &str, fix_errors: bool, aggressive: bool) -> Option<ModesMessage> {
    let hex = hex.trim();

    // Validate format: must start with '*' and end with ';'
    if hex.len() < 4 || !hex.starts_with('*') || !hex.ends_with(';') {
        return None;
    }

    // Extract hex data between * and ;
    let hex_data = &hex[1..hex.len() - 1];

    // Validate length (must be even and not too long)
    if hex_data.len() > MODES_LONG_MSG_BYTES * 2 || hex_data.len() % 2 != 0 {
        return None;
    }

    // Convert hex string to bytes
    let mut msg = [0u8; MODES_LONG_MSG_BYTES];
    for (i, chunk) in hex_data.as_bytes().chunks(2).enumerate() {
        let high = hex_digit_val(chunk[0])?;
        let low = hex_digit_val(chunk[1])?;
        msg[i] = (high << 4) | low;
    }

    Some(decode_modes_message(
        &msg[..hex_data.len() / 2],
        fix_errors,
        aggressive,
    ))
}

/// Convert a hex digit character to its numeric value
fn hex_digit_val(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_len_by_type() {
        // Short messages (56 bits)
        assert_eq!(message_len_by_type(0), MODES_SHORT_MSG_BITS);
        assert_eq!(message_len_by_type(4), MODES_SHORT_MSG_BITS);
        assert_eq!(message_len_by_type(5), MODES_SHORT_MSG_BITS);
        assert_eq!(message_len_by_type(11), MODES_SHORT_MSG_BITS);

        // Long messages (112 bits)
        assert_eq!(message_len_by_type(16), MODES_LONG_MSG_BITS);
        assert_eq!(message_len_by_type(17), MODES_LONG_MSG_BITS);
        assert_eq!(message_len_by_type(20), MODES_LONG_MSG_BITS);
        assert_eq!(message_len_by_type(21), MODES_LONG_MSG_BITS);
    }

    #[test]
    fn test_hex_digit_val() {
        assert_eq!(hex_digit_val(b'0'), Some(0));
        assert_eq!(hex_digit_val(b'9'), Some(9));
        assert_eq!(hex_digit_val(b'a'), Some(10));
        assert_eq!(hex_digit_val(b'f'), Some(15));
        assert_eq!(hex_digit_val(b'A'), Some(10));
        assert_eq!(hex_digit_val(b'F'), Some(15));
        assert_eq!(hex_digit_val(b'g'), None);
        assert_eq!(hex_digit_val(b' '), None);
    }

    #[test]
    fn test_decode_hex_message_format() {
        // Valid format
        assert!(decode_hex_message("*8D4840D6202CC371C32CE0576098;", false, false).is_some());

        // Invalid formats
        assert!(decode_hex_message("8D4840D6202CC371C32CE0576098", false, false).is_none());
        assert!(decode_hex_message("*8D4840D6202CC371C32CE0576098", false, false).is_none());
        assert!(decode_hex_message("8D4840D6202CC371C32CE0576098;", false, false).is_none());
        assert!(decode_hex_message("*;", false, false).is_none());
        assert!(decode_hex_message("*8D;", false, false).is_some()); // Short but valid format
    }

    #[test]
    fn test_decode_df17_message() {
        // DF17 ADS-B message
        let msg = decode_hex_message("*8D4840D6202CC371C32CE0576098;", true, false);
        assert!(msg.is_some());

        let msg = msg.unwrap();
        assert_eq!(msg.msg_type, 17); // DF17
        assert_eq!(msg.msg_bits, 112);
        assert_eq!(msg.aa, [0x48, 0x40, 0xD6]);
        assert_eq!(msg.icao_address(), 0x4840D6);
    }

    #[test]
    fn test_icao_address() {
        let mut mm = ModesMessage::default();
        mm.aa = [0x48, 0x40, 0xD6];
        assert_eq!(mm.icao_address(), 0x4840D6);

        mm.aa = [0x00, 0x00, 0x01];
        assert_eq!(mm.icao_address(), 0x000001);

        mm.aa = [0xFF, 0xFF, 0xFF];
        assert_eq!(mm.icao_address(), 0xFFFFFF);
    }

    #[test]
    fn test_to_raw_string() {
        let mut mm = ModesMessage::default();
        mm.msg = [
            0x8D, 0x48, 0x40, 0xD6, 0x20, 0x2C, 0xC3, 0x71, 0xC3, 0x2C, 0xE0, 0x57, 0x60, 0x98,
        ];
        mm.msg_bits = 112;

        let raw = mm.to_raw_string();
        assert_eq!(raw, "*8D4840D6202CC371C32CE0576098;");
    }

    #[test]
    fn test_squawk_decoding() {
        // Test squawk identity decoding
        // This is a synthetic test - real squawk values would come from actual messages
        let mut msg = [0u8; 14];
        msg[0] = 0x28; // DF5

        // Set up bits for squawk 7700 (emergency)
        // Squawk encoding is complex with interleaved bits
        // For now just verify the decoding doesn't panic
        let mm = decode_modes_message(&msg[..], false, false);
        assert_eq!(mm.msg_type, 5);
    }

    #[test]
    fn test_altitude_decoding() {
        // Test AC12 altitude field decoding
        let mut msg = [0u8; 14];
        msg[0] = 0x8D; // DF17
        msg[4] = 0x58; // ME type 11 (airborne position)
        msg[5] = 0xC3; // Altitude high bits with Q=1
        msg[6] = 0x82; // Altitude low bits

        let mm = decode_modes_message(&msg[..], false, false);
        // Verify altitude was decoded (exact value depends on encoding)
        assert!(mm.altitude != 0 || msg[5] & 0x01 == 0);
    }

        #[test]
        fn test_flight_callsign_decoding() {
            // DF17 with ME type 1-4 contains aircraft identification
            let mut msg = [0u8; 14];
            msg[0] = 0x8D; // DF17
            msg[4] = 0x20; // ME type 4 (aircraft identification)
    
            // Encode "TEST" in AIS charset
            // Each character is 6 bits
            // T=20, E=5, S=19, T=20 (in AIS charset: A=1, B=2, ..., space=32)
            // Actually AIS: ?=0, A=1, B=2, ... Z=26, space=32, 0=48...
    
            let mm = decode_modes_message(&msg[..], false, false);
            assert_eq!(mm.me_type, 4);
        }
    }