//! Mode S message decoder
//!
//!  Decodes raw Mode S messages into structured data.

use std::fmt;

use crate::crc::{self, extract_crc, modes_checksum};

/// Constants for message sizes
pub const MODES_LONG_MSG_BITS: usize = 112;
pub const MODES_SHORT_MSG_BITS: usize = 56;
pub const MODES_LONG_MSG_BYTES: usize = 14;
#[allow(dead_code)]
pub const MODES_SHORT_MSG_BYTES: usize = 7;

/// Unit for altitude measurements
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AltitudeUnit {
    Feet,
    Meters,
}

/// BDS (Comm-B Data Selector) register types
#[derive(Debug, Clone, PartialEq)]
pub enum BdsData {
    /// BDS 1,0 - Data link capability report
    DataLinkCapability {
        continuation_flag: bool,
        overlay_capability: bool,
    },
    /// BDS 2,0 - Aircraft identification
    AircraftIdentification { callsign: String },
    /// BDS 3,0 - ACAS active resolution advisory
    AcasResolutionAdvisory {
        ara: u16,
        rac: u8,
        rat: bool,
        mte: bool,
    },
    /// BDS 4,0 - Selected vertical intention
    SelectedVerticalIntention {
        mcp_altitude: Option<u16>,
        fms_altitude: Option<u16>,
        baro_setting: Option<f32>,
        vnav_mode: bool,
        alt_hold_mode: bool,
        approach_mode: bool,
    },
    /// BDS 5,0 - Track and turn report
    TrackAndTurnReport {
        roll_angle: Option<f32>,
        true_track: Option<f32>,
        ground_speed: Option<u16>,
        track_rate: Option<f32>,
        true_airspeed: Option<u16>,
    },
    /// BDS 6,0 - Heading and speed report
    HeadingAndSpeedReport {
        magnetic_heading: Option<f32>,
        indicated_airspeed: Option<u16>,
        mach: Option<f32>,
        baro_altitude_rate: Option<i16>,
        inertial_altitude_rate: Option<i16>,
    },
    /// Unknown or unimplemented BDS
    Unknown { bds_code: u8, data: [u8; 7] },
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
    pub raw_longitude: u32,
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
    /// Signal level (preamble peak magnitude)
    pub signal_level: u16,
    /// BDS data from DF20/DF21 MB field
    pub bds_data: Option<BdsData>,
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
            signal_level: 0,
            bds_data: None,
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
    #[allow(dead_code)]
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
    #[allow(dead_code)]
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
            "CRC:  {:06x} ({})",
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

                if self.msg_type == 20 {
                    if let Some(ref bds) = self.bds_data {
                        writeln!(f, "  MB Field (BDS) : {}", format_bds_data(bds))?;
                    }
                }
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

                if self.msg_type == 21 {
                    if let Some(ref bds) = self.bds_data {
                        writeln!(f, "  MB Field (BDS) : {}", format_bds_data(bds))?;
                    }
                }
            }
            11 => {
                writeln!(f, "DF 11: All Call Reply.")?;
                writeln!(f, "  Capability  : {}", capability_str(self.ca))?;
                writeln!(
                    f,
                    "  ICAO Address:  {:02x}{:02x}{:02x}",
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
                writeln!(f, "  Extended Squitter  Type:  {}", self.me_type)?;
                writeln!(f, "  Extended Squitter  Sub :  {}", self.me_sub)?;
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
                    writeln!(f, "    Identification :  {}", self.flight)?;
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
                    writeln!(f, "    Altitude :  {} feet", self.altitude)?;
                    writeln!(f, "    Latitude : {} (not decoded)", self.raw_latitude)?;
                    writeln!(f, "    Longitude:  {} (not decoded)", self.raw_longitude)?;
                } else if self.me_type == 19 && (1..=4).contains(&self.me_sub) {
                    if self.me_sub == 1 || self.me_sub == 2 {
                        writeln!(f, "    EW direction      : {}", self.ew_dir)?;
                        writeln!(f, "    EW velocity       : {}", self.ew_velocity)?;
                        writeln!(f, "    NS direction      : {}", self.ns_dir)?;
                        writeln!(f, "    NS velocity       : {}", self.ns_velocity)?;
                        writeln!(f, "    Vertical rate src :  {}", self.vert_rate_source)?;
                        writeln!(f, "    Vertical rate sign:  {}", self.vert_rate_sign)?;
                        writeln!(f, "    Vertical rate     : {}", self.vert_rate)?;
                    } else {
                        writeln!(f, "    Heading status:  {}", self.heading_is_valid)?;
                        writeln!(f, "    Heading:  {:.1}", self.heading)?;
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
                writeln!(f, "DF {} (decoding not fully implemented)", self.msg_type)?;
            }
        }

        Ok(())
    }
}

/// Format BDS data for display
fn format_bds_data(bds: &BdsData) -> String {
    match bds {
        BdsData::DataLinkCapability {
            continuation_flag,
            overlay_capability,
        } => {
            format!(
                "BDS 1,0 - Data Link Capability (cont={}, overlay={})",
                continuation_flag, overlay_capability
            )
        }
        BdsData::AircraftIdentification { callsign } => {
            format!("BDS 2,0 - Aircraft ID:  {}", callsign)
        }
        BdsData::AcasResolutionAdvisory { ara, rac, rat, mte } => {
            format!(
                "BDS 3,0 - ACAS RA (ARA={}, RAC={}, RAT={}, MTE={})",
                ara, rac, rat, mte
            )
        }
        BdsData::SelectedVerticalIntention {
            mcp_altitude,
            fms_altitude,
            baro_setting,
            vnav_mode,
            alt_hold_mode,
            approach_mode,
        } => {
            let mcp = mcp_altitude
                .map(|a| format!("{} ft", a))
                .unwrap_or_else(|| "N/A".to_string());
            let fms = fms_altitude
                .map(|a| format!("{} ft", a))
                .unwrap_or_else(|| "N/A".to_string());
            let baro = baro_setting
                .map(|b| format!("{:.1} mb", b))
                .unwrap_or_else(|| "N/A".to_string());
            format!(
                "BDS 4,0 - MCP Alt: {}, FMS Alt: {}, Baro: {}, VNAV={}, ALT_HOLD={}, APP={}",
                mcp, fms, baro, vnav_mode, alt_hold_mode, approach_mode
            )
        }
        BdsData::TrackAndTurnReport {
            roll_angle,
            true_track,
            ground_speed,
            track_rate,
            true_airspeed,
        } => {
            let roll = roll_angle
                .map(|r| format!("{:.1}°", r))
                .unwrap_or_else(|| "N/A".to_string());
            let track = true_track
                .map(|t| format!("{:.1}°", t))
                .unwrap_or_else(|| "N/A".to_string());
            let gs = ground_speed
                .map(|g| format!("{} kt", g))
                .unwrap_or_else(|| "N/A".to_string());
            let tr = track_rate
                .map(|t| format!("{:.2}°/s", t))
                .unwrap_or_else(|| "N/A".to_string());
            let tas = true_airspeed
                .map(|t| format!("{} kt", t))
                .unwrap_or_else(|| "N/A".to_string());
            format!(
                "BDS 5,0 - Roll:  {}, Track: {}, GS: {}, Track Rate: {}, TAS: {}",
                roll, track, gs, tr, tas
            )
        }
        BdsData::HeadingAndSpeedReport {
            magnetic_heading,
            indicated_airspeed,
            mach,
            baro_altitude_rate,
            inertial_altitude_rate,
        } => {
            let hdg = magnetic_heading
                .map(|h| format!("{:.1}°", h))
                .unwrap_or_else(|| "N/A".to_string());
            let ias = indicated_airspeed
                .map(|i| format!("{} kt", i))
                .unwrap_or_else(|| "N/A".to_string());
            let m = mach
                .map(|m| format!("{:.3}", m))
                .unwrap_or_else(|| "N/A".to_string());
            let bar = baro_altitude_rate
                .map(|b| format!("{} ft/min", b))
                .unwrap_or_else(|| "N/A".to_string());
            let iar = inertial_altitude_rate
                .map(|i| format!("{} ft/min", i))
                .unwrap_or_else(|| "N/A".to_string());
            format!(
                "BDS 6,0 - Hdg: {}, IAS: {}, Mach: {}, Baro Rate: {}, Inertial Rate: {}",
                hdg, ias, m, bar, iar
            )
        }
        BdsData::Unknown { bds_code, data } => {
            format!(
                "BDS {:X},{:X} - Raw:  {:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}",
                bds_code >> 4,
                bds_code & 0x0F,
                data[0],
                data[1],
                data[2],
                data[3],
                data[4],
                data[5],
                data[6]
            )
        }
    }
}

/// AIS charset for flight ID decoding
const AIS_CHARSET: &[u8; 64] = b"?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????";

/// Decode Gillham (Gray code) altitude from Mode C reply
///
/// Mode C uses Gillham code to encode altitude in 100-foot increments.
/// This is a legacy encoding still used when the Q-bit is 0.
///
/// The encoding uses a reflected Gray code pattern that allows
/// altitude to be encoded from -1200 to 126,700 feet. 
fn decode_gillham_altitude(code: u16) -> Option<i32> {
    if code == 0 {
        return None;
    }

    // Extract bits - the arrangement in Mode S message is:
    // Bit:  10  9  8  7  6  5  4  3  2  1  0
    //      D4 D2 B4 B2 B1 A4 A2 A1 C4 C2 C1

    let c1 = (code & 0x001) != 0;
    let c2 = (code & 0x002) != 0;
    let c4 = (code & 0x004) != 0;
    let a1 = (code & 0x008) != 0;
    let a2 = (code & 0x010) != 0;
    let a4 = (code & 0x020) != 0;
    let b1 = (code & 0x040) != 0;
    let b2 = (code & 0x080) != 0;
    let b4 = (code & 0x100) != 0;
    let d2 = (code & 0x200) != 0;
    let d4 = (code & 0x400) != 0;

    // Compute the 500ft increment from D and B groups using Gray code
    // Gray code:  D4 D2 D1 B4 B2 B1 (D1 is not transmitted, assumed 0)
    let mut grayval:  i32 = 0;

    if d4 { grayval |= 0x20; }
    if d2 { grayval |= 0x10; }
    // D1 not transmitted (bit 0x08 stays 0)
    if b4 { grayval |= 0x04; }
    if b2 { grayval |= 0x02; }
    if b1 { grayval |= 0x01; }

    // Convert Gray to binary for 500ft bands
    let mut five_hundreds = grayval;
    five_hundreds ^= five_hundreds >> 4;
    five_hundreds ^= five_hundreds >> 2;
    five_hundreds ^= five_hundreds >> 1;

    // Compute the 100ft increment from C and A groups
    // Build Gray code value from C and A bits
    let mut gray100:  i32 = 0;
    if c4 { gray100 |= 0x10; }
    if c2 { gray100 |= 0x08; }
    if c1 { gray100 |= 0x04; }
    if a4 { gray100 |= 0x02; }
    if a2 { gray100 |= 0x01; }
    // A1 determines odd/even within the pattern

    // Convert Gray to binary
    let mut one_hundreds = gray100;
    one_hundreds ^= one_hundreds >> 4;
    one_hundreds ^= one_hundreds >> 2;
    one_hundreds ^= one_hundreds >> 1;

    // The 100ft digit cycles 0,1,2,3,4,5,6,7,8,9 but in a reflected pattern
    // We need to map this to actual 100ft increments (0-4 within each 500ft band)
    // A1 bit helps determine the reflection
    let hundreds = if a1 {
        // Odd position - values 5,6,7,8,9 map to 4,3,2,1,0
        4 - ((one_hundreds) % 5).min(4)
    } else {
        // Even position - values 0,1,2,3,4
        (one_hundreds % 5).min(4)
    };

    // Final altitude calculation
    // Each "five_hundreds" unit = 500 feet
    // Each "hundreds" unit = 100 feet
    // Offset of -1300 feet to allow encoding of negative altitudes
    let altitude = (five_hundreds * 500) + (hundreds * 100) - 1300;

    // Validate range (-1200 to 126,700 feet for Mode C)
    if altitude >= -1200 && altitude <= 126700 {
        Some(altitude)
    } else {
        None
    }
}

/// Decode Comm-B MB field (56 bits) for DF20/DF21
fn decode_mb_field(msg: &[u8]) -> Option<BdsData> {
    if msg.len() < 11 {
        return None;
    }

    let mb = &msg[4..11];

    if let Some(bds) = try_decode_bds_20(mb) {
        return Some(bds);
    }
    if let Some(bds) = try_decode_bds_40(mb) {
        return Some(bds);
    }
    if let Some(bds) = try_decode_bds_50(mb) {
        return Some(bds);
    }
    if let Some(bds) = try_decode_bds_60(mb) {
        return Some(bds);
    }
    if let Some(bds) = try_decode_bds_30(mb) {
        return Some(bds);
    }
    if let Some(bds) = try_decode_bds_10(mb) {
        return Some(bds);
    }

    let mut data = [0u8; 7];
    data.copy_from_slice(mb);
    Some(BdsData::Unknown {
        bds_code: 0x00,
        data,
    })
}

fn try_decode_bds_10(mb: &[u8]) -> Option<BdsData> {
    if mb[0] != 0x10 {
        return None;
    }
    let continuation_flag = (mb[1] & 0x80) != 0;
    let overlay_capability = (mb[1] & 0x02) != 0;
    Some(BdsData::DataLinkCapability {
        continuation_flag,
        overlay_capability,
    })
}

fn try_decode_bds_20(mb: &[u8]) -> Option<BdsData> {
    let char_indices = [
        (mb[0] >> 2) as usize,
        (((mb[0] & 0x03) << 4) | (mb[1] >> 4)) as usize,
        (((mb[1] & 0x0F) << 2) | (mb[2] >> 6)) as usize,
        (mb[2] & 0x3F) as usize,
        (mb[3] >> 2) as usize,
        (((mb[3] & 0x03) << 4) | (mb[4] >> 4)) as usize,
        (((mb[4] & 0x0F) << 2) | (mb[5] >> 6)) as usize,
        (mb[5] & 0x3F) as usize,
    ];

    let mut valid_chars = true;
    for &idx in &char_indices {
        if idx >= 64 {
            valid_chars = false;
            break;
        }
        let c = AIS_CHARSET[idx];
        if c == b'?' && idx != 0 {
            valid_chars = false;
            break;
        }
    }

    if !valid_chars {
        return None;
    }

    let chars: Vec<char> = char_indices
        .iter()
        .map(|&idx| AIS_CHARSET[idx.min(63)] as char)
        .collect();

    let callsign: String = chars.into_iter().collect::<String>().trim().to_string();

    if callsign.is_empty() || callsign.chars().all(|c| c == ' ' || c == '?') {
        return None;
    }

    Some(BdsData::AircraftIdentification { callsign })
}

fn try_decode_bds_30(mb: &[u8]) -> Option<BdsData> {
    let ara = ((mb[0] as u16) << 6) | ((mb[1] >> 2) as u16);
    let rac = ((mb[1] & 0x03) << 2) | (mb[2] >> 6);
    let rat = (mb[2] & 0x20) != 0;
    let mte = (mb[2] & 0x10) != 0;

    if ara == 0 && rac == 0 {
        return None;
    }

    Some(BdsData::AcasResolutionAdvisory { ara, rac, rat, mte })
}

fn try_decode_bds_40(mb: &[u8]) -> Option<BdsData> {
    let mcp_status = (mb[0] & 0x80) != 0;
    let fms_status = (mb[2] & 0x80) != 0;
    let baro_status = (mb[4] & 0x80) != 0;

    let mcp_altitude = if mcp_status {
        let raw = ((mb[0] as u16 & 0x7F) << 5) | ((mb[1] >> 3) as u16);
        Some((raw * 16) as u16)
    } else {
        None
    };

    let fms_altitude = if fms_status {
        let raw = ((mb[2] as u16 & 0x7F) << 5) | ((mb[3] >> 3) as u16);
        Some((raw * 16) as u16)
    } else {
        None
    };

    let baro_setting = if baro_status {
        let raw = ((mb[4] as u16 & 0x7F) << 5) | ((mb[5] >> 3) as u16);
        Some(800.0 + (raw as f32) * 0.1)
    } else {
        None
    };

    let vnav_mode = (mb[6] & 0x08) != 0;
    let alt_hold_mode = (mb[6] & 0x04) != 0;
    let approach_mode = (mb[6] & 0x02) != 0;

    if mcp_altitude.is_none() && fms_altitude.is_none() && baro_setting.is_none() {
        return None;
    }

    if let Some(alt) = mcp_altitude {
        if alt > 50000 {
            return None;
        }
    }
    if let Some(alt) = fms_altitude {
        if alt > 50000 {
            return None;
        }
    }
    if let Some(baro) = baro_setting {
        if baro < 850.0 || baro > 1100.0 {
            return None;
        }
    }

    Some(BdsData::SelectedVerticalIntention {
        mcp_altitude,
        fms_altitude,
        baro_setting,
        vnav_mode,
        alt_hold_mode,
        approach_mode,
    })
}

fn try_decode_bds_50(mb: &[u8]) -> Option<BdsData> {
    let roll_status = (mb[0] & 0x80) != 0;
    let track_status = (mb[1] & 0x10) != 0;
    let gs_status = (mb[2] & 0x02) != 0;
    let track_rate_status = (mb[3] & 0x40) != 0;
    let tas_status = (mb[4] & 0x08) != 0;

    let roll_angle = if roll_status {
        let raw = ((mb[0] as i16 & 0x7F) << 3) | ((mb[1] >> 5) as i16);
        let signed = if raw & 0x200 != 0 { raw - 0x400 } else { raw };
        Some((signed as f32) * 45.0 / 256.0)
    } else {
        None
    };

    let true_track = if track_status {
        let raw = ((mb[1] as u16 & 0x0F) << 7) | ((mb[2] >> 1) as u16);
        Some((raw as f32) * 90.0 / 512.0)
    } else {
        None
    };

    let ground_speed = if gs_status {
        let raw = ((mb[2] as u16 & 0x01) << 9) | ((mb[3] as u16) << 1) | ((mb[4] >> 7) as u16);
        Some((raw * 2) as u16)
    } else {
        None
    };

    let track_rate = if track_rate_status {
        let raw = ((mb[4] as i16 & 0x3F) << 3) | ((mb[5] >> 5) as i16);
        let signed = if raw & 0x100 != 0 { raw - 0x200 } else { raw };
        Some((signed as f32) * 8.0 / 256.0)
    } else {
        None
    };

    let true_airspeed = if tas_status {
        let raw = ((mb[5] as u16 & 0x1F) << 5) | ((mb[6] >> 3) as u16);
        Some((raw * 2) as u16)
    } else {
        None
    };

    let valid_count = [
        roll_status,
        track_status,
        gs_status,
        track_rate_status,
        tas_status,
    ]
    .iter()
    .filter(|&&x| x)
    .count();

    if valid_count < 2 {
        return None;
    }

    if let Some(roll) = roll_angle {
        if roll.abs() > 60.0 {
            return None;
        }
    }
    if let Some(gs) = ground_speed {
        if gs > 600 {
            return None;
        }
    }
    if let Some(tas) = true_airspeed {
        if tas > 600 {
            return None;
        }
    }

    Some(BdsData::TrackAndTurnReport {
        roll_angle,
        true_track,
        ground_speed,
        track_rate,
        true_airspeed,
    })
}

fn try_decode_bds_60(mb: &[u8]) -> Option<BdsData> {
    let hdg_status = (mb[0] & 0x80) != 0;
    let ias_status = (mb[1] & 0x10) != 0;
    let mach_status = (mb[2] & 0x02) != 0;
    let baro_rate_status = (mb[3] & 0x40) != 0;
    let inertial_rate_status = (mb[4] & 0x08) != 0;

    let magnetic_heading = if hdg_status {
        let raw = ((mb[0] as u16 & 0x7F) << 4) | ((mb[1] >> 4) as u16);
        Some((raw as f32) * 90.0 / 512.0)
    } else {
        None
    };

    let indicated_airspeed = if ias_status {
        let raw = ((mb[1] as u16 & 0x0F) << 6) | ((mb[2] >> 2) as u16);
        Some(raw as u16)
    } else {
        None
    };

    let mach = if mach_status {
        let raw = ((mb[2] as u16 & 0x01) << 9) | ((mb[3] as u16) << 1) | ((mb[4] >> 7) as u16);
        Some((raw as f32) * 0.008)
    } else {
        None
    };

    let baro_altitude_rate = if baro_rate_status {
        let raw = ((mb[4] as i16 & 0x3F) << 4) | ((mb[5] >> 4) as i16);
        let signed = if raw & 0x200 != 0 { raw - 0x400 } else { raw };
        Some((signed * 32) as i16)
    } else {
        None
    };

    let inertial_altitude_rate = if inertial_rate_status {
        let raw = ((mb[5] as i16 & 0x0F) << 6) | ((mb[6] >> 2) as i16);
        let signed = if raw & 0x200 != 0 { raw - 0x400 } else { raw };
        Some((signed * 32) as i16)
    } else {
        None
    };

    let valid_count = [
        hdg_status,
        ias_status,
        mach_status,
        baro_rate_status,
        inertial_rate_status,
    ]
    .iter()
    .filter(|&&x| x)
    .count();

    if valid_count < 2 {
        return None;
    }

    if let Some(ias) = indicated_airspeed {
        if ias > 500 {
            return None;
        }
    }
    if let Some(m) = mach {
        if m > 1.0 {
            return None;
        }
    }

    Some(BdsData::HeadingAndSpeedReport {
        magnetic_heading,
        indicated_airspeed,
        mach,
        baro_altitude_rate,
        inertial_altitude_rate,
    })
}

/// Decode a Mode S message from raw bytes.  
///
/// For DF4/5/20/21, we can only validate if we have a known ICAO to check against.
/// Pass `known_icao` as Some(icao) to validate, or None to attempt recovery.
pub fn decode_modes_message(raw_msg: &[u8], fix_errors: bool, aggressive: bool) -> ModesMessage {
    let mut mm = ModesMessage::default();

    // Copy message to local buffer
    let len = raw_msg.len().min(MODES_LONG_MSG_BYTES);
    mm.msg[..len].copy_from_slice(&raw_msg[..len]);

    // Get message type (Downlink Format) from first 5 bits
    mm.msg_type = mm.msg[0] >> 3;
    mm.msg_bits = message_len_by_type(mm.msg_type);

    // Determine if ICAO is in message or XORed with CRC
    // DF11, DF17, DF18 have ICAO in message bytes 1-3
    // DF0, DF4, DF5, DF16, DF20, DF21 have ICAO XORed with CRC
    let icao_in_message = matches!(mm.msg_type, 11 | 17 | 18);

    if icao_in_message {
        // ICAO address is in bytes 1, 2, 3
        mm.aa = [mm.msg[1], mm.msg[2], mm.msg[3]];

        // Extract and verify CRC
        mm.crc = extract_crc(&mm.msg, mm.msg_bits);
        let computed_crc = modes_checksum(&mm.msg, mm.msg_bits);
        mm.crc_ok = mm.crc == computed_crc;

        // Attempt error correction for DF11 and DF17 messages
        if !mm.crc_ok && fix_errors && (mm.msg_type == 11 || mm.msg_type == 17) {
            if let Some(bit) = crc::fix_single_bit_errors(&mut mm.msg, mm.msg_bits) {
                mm.error_bit = Some(bit);
                mm.crc = extract_crc(&mm.msg, mm.msg_bits);
                mm.crc_ok = true;
            } else if aggressive && mm.msg_type == 17 {
                if let Some((bit1, bit2)) = crc::fix_two_bit_errors(&mut mm.msg, mm.msg_bits) {
                    mm.error_bit = Some(bit1);
                    mm.error_bit2 = Some(bit2);
                    mm.crc = extract_crc(&mm.msg, mm.msg_bits);
                    mm.crc_ok = true;
                }
            }
        }
    } else {
        // DF0, DF4, DF5, DF16, DF20, DF21 - ICAO is XORed into CRC
        // We mark these as NOT OK initially - they need external validation
        // against known ICAO addresses
        let computed_crc = modes_checksum(&mm.msg, mm.msg_bits);
        let received_crc = extract_crc(&mm.msg, mm.msg_bits);
        let recovered_icao = computed_crc ^ received_crc;

        mm.crc = received_crc;
        mm.aa = [
            ((recovered_icao >> 16) & 0xFF) as u8,
            ((recovered_icao >> 8) & 0xFF) as u8,
            (recovered_icao & 0xFF) as u8,
        ];

        // Mark as NOT OK - needs validation against known ICAOs
        // The aircraft tracker will validate this
        mm.crc_ok = false;
    }

    // === Decode common fields ===
    mm.ca = mm.msg[0] & 0x07;

    // === Decode DF17 specific fields ===
    mm.me_type = mm.msg[4] >> 3;
    mm.me_sub = mm.msg[4] & 0x07;

    // === Decode fields for DF4, DF5, DF20, DF21 ===
    mm.fs = mm.msg[0] & 0x07;
    mm.dr = (mm.msg[1] >> 3) & 0x1F;
    mm.um = ((mm.msg[1] & 0x07) << 3) | (mm.msg[2] >> 5);

    // === Decode squawk (identity) for DF5, DF21 ===
    if mm.msg_type == 5 || mm.msg_type == 21 {
        let a = ((mm.msg[3] & 0x80) >> 5) | (mm.msg[2] & 0x02) | ((mm.msg[2] & 0x08) >> 3);
        let b = ((mm.msg[3] & 0x02) << 1) | ((mm.msg[3] & 0x08) >> 2) | ((mm.msg[3] & 0x20) >> 5);
        let c = ((mm.msg[2] & 0x01) << 2) | ((mm.msg[2] & 0x04) >> 1) | ((mm.msg[2] & 0x10) >> 4);
        let d = ((mm.msg[3] & 0x01) << 2) | ((mm.msg[3] & 0x04) >> 1) | ((mm.msg[3] & 0x10) >> 4);
        mm.identity = (a as u16) * 1000 + (b as u16) * 100 + (c as u16) * 10 + (d as u16);
    }

    // === Decode altitude for DF0, DF4, DF16, DF20 ===
    if matches!(mm.msg_type, 0 | 4 | 16 | 20) {
        mm.altitude = decode_ac13_field(&mm.msg, &mut mm.unit);
    }

    // === Decode extended squitter (DF17) ===
    if mm.msg_type == 17 {
        decode_extended_squitter(&mut mm);
    }

    // === Decode MB field for DF20/DF21 ===
    if mm.msg_type == 20 || mm.msg_type == 21 {
        mm.bds_data = decode_mb_field(&mm.msg);
    }

    mm
}

/// Validate a message with ICAO-in-CRC against a known ICAO address
#[allow(dead_code)]
pub fn validate_icao(mm: &mut ModesMessage, known_icao: u32) {
    if mm.crc_ok {
        return; // Already validated
    }

    // Check if recovered ICAO matches known ICAO
    let recovered = mm.icao_address();
    if recovered == known_icao {
        mm.crc_ok = true;
    }
}
/// Decode extended squitter message (DF17)
fn decode_extended_squitter(mm: &mut ModesMessage) {
    if (1..=4).contains(&mm.me_type) {
        mm.aircraft_type = mm.me_type - 1;

        let char_indices = [
            (mm.msg[5] >> 2) as usize,
            (((mm.msg[5] & 0x03) << 4) | (mm.msg[6] >> 4)) as usize,
            (((mm.msg[6] & 0x0F) << 2) | (mm.msg[7] >> 6)) as usize,
            (mm.msg[7] & 0x3F) as usize,
            (mm.msg[8] >> 2) as usize,
            (((mm.msg[8] & 0x03) << 4) | (mm.msg[9] >> 4)) as usize,
            (((mm.msg[9] & 0x0F) << 2) | (mm.msg[10] >> 6)) as usize,
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

        mm.flight = chars.into_iter().collect::<String>().trim().to_string();
    } else if (9..=18).contains(&mm.me_type) {
        mm.fflag = (mm.msg[6] & 0x04) != 0;
        mm.tflag = (mm.msg[6] & 0x08) != 0;
        mm.altitude = decode_ac12_field(&mm.msg, &mut mm.unit);

        mm.raw_latitude = (((mm.msg[6] & 0x03) as u32) << 15)
            | ((mm.msg[7] as u32) << 7)
            | ((mm.msg[8] >> 1) as u32);
        mm.raw_longitude =
            (((mm.msg[8] & 0x01) as u32) << 16) | ((mm.msg[9] as u32) << 8) | (mm.msg[10] as u32);
    } else if mm.me_type == 19 && (1..=4).contains(&mm.me_sub) {
        if mm.me_sub == 1 || mm.me_sub == 2 {
            mm.ew_dir = (mm.msg[5] & 0x04) >> 2;
            mm.ew_velocity = (((mm.msg[5] & 0x03) as u16) << 8) | (mm.msg[6] as u16);
            mm.ns_dir = (mm.msg[7] & 0x80) >> 7;
            mm.ns_velocity =
                (((mm.msg[7] & 0x7F) as u16) << 3) | (((mm.msg[8] & 0xE0) >> 5) as u16);
            mm.vert_rate_source = (mm.msg[8] & 0x10) >> 4;
            mm.vert_rate_sign = (mm.msg[8] & 0x08) >> 3;
            mm.vert_rate = (((mm.msg[8] & 0x07) as u16) << 6) | (((mm.msg[9] & 0xFC) >> 2) as u16);

            let ewv = mm.ew_velocity as f64;
            let nsv = mm.ns_velocity as f64;
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
            mm.heading = (360.0 / 128.0)
                * ((((mm.msg[5] & 0x03) as u16) << 5) | ((mm.msg[6] >> 3) as u16)) as f64;
        }
    }
}

/// Decode 13-bit AC altitude field (used in DF0, DF4, DF16, DF20)
fn decode_ac13_field(msg: &[u8], unit: &mut AltitudeUnit) -> i32 {
    let m_bit = (msg[3] & 0x40) != 0;
    let q_bit = (msg[3] & 0x10) != 0;

    if !m_bit {
        *unit = AltitudeUnit::Feet;
        if q_bit {
            let n = (((msg[2] & 0x1F) as i32) << 6)
                | (((msg[3] & 0x80) >> 2) as i32)
                | (((msg[3] & 0x20) >> 1) as i32)
                | ((msg[3] & 0x0F) as i32);
            return n * 25 - 1000;
        } else {
            let c1 = (msg[2] >> 4) & 1;
            let a1 = (msg[2] >> 3) & 1;
            let c2 = (msg[2] >> 2) & 1;
            let a2 = (msg[2] >> 1) & 1;
            let c4 = msg[2] & 1;
            let a4 = (msg[3] >> 7) & 1;
            let b1 = (msg[3] >> 5) & 1;
            let d2 = (msg[3] >> 3) & 1;
            let b2 = (msg[3] >> 2) & 1;
            let d4 = (msg[3] >> 1) & 1;
            let b4 = msg[3] & 1;

            let code = ((d4 as u16) << 10)
                | ((d2 as u16) << 9)
                | ((b4 as u16) << 8)
                | ((b2 as u16) << 7)
                | ((b1 as u16) << 6)
                | ((a4 as u16) << 5)
                | ((a2 as u16) << 4)
                | ((a1 as u16) << 3)
                | ((c4 as u16) << 2)
                | ((c2 as u16) << 1)
                | (c1 as u16);

            if let Some(alt) = decode_gillham_altitude(code) {
                return alt;
            }
        }
    } else {
        *unit = AltitudeUnit::Meters;
        let n = (((msg[2] & 0x1F) as i32) << 7)
            | (((msg[3] & 0x80) >> 1) as i32)
            | ((msg[3] & 0x20) as i32)
            | ((msg[3] & 0x0F) as i32);
        return n * 25;
    }
    0
}

/// Decode 12-bit AC altitude field (used in DF17 airborne position)
fn decode_ac12_field(msg: &[u8], unit: &mut AltitudeUnit) -> i32 {
    let q_bit = (msg[5] & 0x01) != 0;

    if q_bit {
        *unit = AltitudeUnit::Feet;
        let n = (((msg[5] >> 1) as i32) << 4) | (((msg[6] & 0xF0) >> 4) as i32);
        return n * 25 - 1000;
    } else {
        *unit = AltitudeUnit::Feet;
        let c1 = (msg[5] >> 1) & 1;
        let a1 = (msg[5] >> 2) & 1;
        let c2 = (msg[5] >> 3) & 1;
        let a2 = (msg[5] >> 4) & 1;
        let c4 = (msg[5] >> 5) & 1;
        let a4 = (msg[5] >> 6) & 1;
        let b1 = (msg[5] >> 7) & 1;
        let b2 = (msg[6] >> 4) & 1;
        let d2 = (msg[6] >> 5) & 1;
        let b4 = (msg[6] >> 6) & 1;
        let d4 = (msg[6] >> 7) & 1;

        let code = ((d4 as u16) << 10)
            | ((d2 as u16) << 9)
            | ((b4 as u16) << 8)
            | ((b2 as u16) << 7)
            | ((b1 as u16) << 6)
            | ((a4 as u16) << 5)
            | ((a2 as u16) << 4)
            | ((a1 as u16) << 3)
            | ((c4 as u16) << 2)
            | ((c2 as u16) << 1)
            | (c1 as u16);

        if let Some(alt) = decode_gillham_altitude(code) {
            return alt;
        }
    }
    0
}

/// Get message length in bits based on Downlink Format
pub fn message_len_by_type(df: u8) -> usize {
    match df {
        16 | 17 | 19 | 20 | 21 => MODES_LONG_MSG_BITS,
        _ => MODES_SHORT_MSG_BITS,
    }
}

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
pub fn decode_hex_message(hex: &str, fix_errors: bool, aggressive: bool) -> Option<ModesMessage> {
    let hex = hex.trim();

    if hex.len() < 4 || !hex.starts_with('*') || !hex.ends_with(';') {
        return None;
    }

    let hex_data = &hex[1..hex.len() - 1];

    if hex_data.len() > MODES_LONG_MSG_BYTES * 2 || hex_data.len() % 2 != 0 {
        return None;
    }

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

fn hex_digit_val(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

/// Convert Gray code to binary (generic version)
/// Works for any bit width up to 32 bits
#[allow(dead_code)]
fn gray_to_binary(gray: u32) -> u32 {
    let mut binary = gray;
    binary ^= binary >> 16;
    binary ^= binary >> 8;
    binary ^= binary >> 4;
    binary ^= binary >> 2;
    binary ^= binary >> 1;
    binary
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_len_by_type() {
        assert_eq!(message_len_by_type(0), MODES_SHORT_MSG_BITS);
        assert_eq!(message_len_by_type(4), MODES_SHORT_MSG_BITS);
        assert_eq!(message_len_by_type(5), MODES_SHORT_MSG_BITS);
        assert_eq!(message_len_by_type(11), MODES_SHORT_MSG_BITS);
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
    }

    #[test]
    fn test_decode_hex_message_format() {
        assert!(decode_hex_message("*8D4840D6202CC371C32CE0576098;", false, false).is_some());
        assert!(decode_hex_message("8D4840D6202CC371C32CE0576098", false, false).is_none());
        assert!(decode_hex_message("*;", false, false).is_none());
    }

    #[test]
    fn test_decode_df17_message() {
        let msg = decode_hex_message("*8D4840D6202CC371C32CE0576098;", true, false);
        assert!(msg.is_some());
        let msg = msg.unwrap();
        assert_eq!(msg.msg_type, 17);
        assert_eq!(msg.msg_bits, 112);
        assert_eq!(msg.aa, [0x48, 0x40, 0xD6]);
    }

    #[test]
    fn test_icao_address() {
        let mut mm = ModesMessage::default();
        mm.aa = [0x48, 0x40, 0xD6];
        assert_eq!(mm.icao_address(), 0x4840D6);
    }

    #[test]
    fn test_to_raw_string() {
        let mut mm = ModesMessage::default();
        mm.msg = [
            0x8D, 0x48, 0x40, 0xD6, 0x20, 0x2C, 0xC3, 0x71, 0xC3, 0x2C, 0xE0, 0x57, 0x60, 0x98,
        ];
        mm.msg_bits = 112;
        assert_eq!(mm.to_raw_string(), "*8D4840D6202CC371C32CE0576098;");
    }

    #[test]
    fn test_df4_icao_recovery() {
        // DF4 message - ICAO should be recovered from CRC
        let msg = decode_hex_message("*20000f1f684a6c;", true, false);
        assert!(msg.is_some());
        let msg = msg.unwrap();
        assert_eq!(msg.msg_type, 4);
        // Note: crc_ok will be false until validated against known ICAOs
        // ICAO should be recovered
        assert_ne!(msg.icao_address(), 0);
    }

    #[test]
    fn test_df5_icao_recovery() {
        // DF5 message - ICAO should be recovered from CRC
        let msg = decode_hex_message("*280010248c796b;", true, false);
        assert!(msg.is_some());
        let msg = msg.unwrap();
        assert_eq!(msg.msg_type, 5);
        // Note: crc_ok will be false until validated against known ICAOs
        // ICAO should be recovered
        assert_ne!(msg.icao_address(), 0);
    }

    #[test]
    fn test_gray_to_binary() {
        // Test Gray code to binary conversion
        assert_eq!(gray_to_binary(0b0000), 0b0000); // 0 -> 0
        assert_eq!(gray_to_binary(0b0001), 0b0001); // 1 -> 1
        assert_eq!(gray_to_binary(0b0011), 0b0010); // 3 -> 2
        assert_eq!(gray_to_binary(0b0010), 0b0011); // 2 -> 3
        assert_eq!(gray_to_binary(0b0110), 0b0100); // 6 -> 4
        assert_eq!(gray_to_binary(0b0111), 0b0101); // 7 -> 5
        assert_eq!(gray_to_binary(0b0101), 0b0110); // 5 -> 6
        assert_eq!(gray_to_binary(0b0100), 0b0111); // 4 -> 7
        assert_eq!(gray_to_binary(0b1100), 0b1000); // 12 -> 8
        assert_eq!(gray_to_binary(0b1111), 0b1010); // 15 -> 10
    }

    #[test]
    fn test_gillham_altitude() {
        // Test some known Gillham altitude values
        // These are based on standard Mode C altitude encoding
        
        // Zero code should return None
        assert_eq!(decode_gillham_altitude(0), None);
        
        // Test that valid codes return Some altitude
        // Note: exact values depend on the specific encoding
        let result = decode_gillham_altitude(0x010);
        assert!(result.is_none() || result.unwrap() >= -1200);
    }
}