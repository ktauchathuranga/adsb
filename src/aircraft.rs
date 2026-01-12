//! Aircraft tracking and position decoding
//!
//!  Maintains a database of recently seen aircraft and decodes CPR positions.

use std::collections::HashMap;
use std::time::{Duration, Instant};

use crate::decoder::{BdsData, ModesMessage};

/// Tracked aircraft data
#[derive(Debug, Clone)]
pub struct Aircraft {
    /// ICAO 24-bit address
    #[allow(dead_code)]
    pub addr: u32,
    /// Hex address string
    pub hex_addr: String,
    /// Flight callsign
    pub flight: String,
    /// Altitude in feet
    pub altitude: i32,
    /// Ground speed in knots
    pub speed: u16,
    /// Track/heading in degrees
    pub track: u16,
    /// Last seen timestamp
    pub seen: Instant,
    /// Message count
    pub messages: u64,
    /// Odd CPR latitude
    pub odd_cprlat: u32,
    /// Odd CPR longitude
    pub odd_cprlon: u32,
    /// Odd CPR timestamp
    pub odd_cprtime: Instant,
    /// Even CPR latitude
    pub even_cprlat: u32,
    /// Even CPR longitude
    pub even_cprlon: u32,
    /// Even CPR timestamp
    pub even_cprtime: Instant,
    /// Decoded latitude
    pub lat: f64,
    /// Decoded longitude
    pub lon: f64,
    /// Roll angle (from BDS 5,0)
    pub roll_angle: Option<f32>,
    /// True airspeed (from BDS 5,0 or 6,0)
    pub true_airspeed: Option<u16>,
    /// Indicated airspeed (from BDS 6,0)
    pub indicated_airspeed: Option<u16>,
    /// Mach number (from BDS 6,0)
    pub mach: Option<f32>,
    /// Magnetic heading (from BDS 6,0)
    pub magnetic_heading: Option<f32>,
    /// Barometric altitude rate (from BDS 6,0)
    pub baro_altitude_rate: Option<i16>,
    /// MCP/FCU selected altitude (from BDS 4,0)
    pub selected_altitude: Option<u16>,
    /// Barometric pressure setting (from BDS 4,0)
    pub baro_setting: Option<f32>,
    /// Squawk code (identity) from DF5/DF21
    pub squawk: u16,
    /// Average signal level (magnitude)
    pub signal_level: u16,
    /// Count of phase-corrected messages
    pub phase_corrections: u32,
}

impl Aircraft {
    pub fn new(addr: u32) -> Self {
        let now = Instant::now();
        Self {
            addr,
            hex_addr: format!("{:06X}", addr),
            flight: String::new(),
            altitude: 0,
            speed: 0,
            track: 0,
            seen: now,
            messages: 0,
            odd_cprlat: 0,
            odd_cprlon: 0,
            odd_cprtime: now,
            even_cprlat: 0,
            even_cprlon: 0,
            even_cprtime: now,
            lat: 0.0,
            lon: 0.0,
            roll_angle: None,
            true_airspeed: None,
            indicated_airspeed: None,
            mach: None,
            magnetic_heading: None,
            baro_altitude_rate: None,
            selected_altitude: None,
            baro_setting: None,
            squawk: 0,
            signal_level: 0,
            phase_corrections: 0,
        }
    }
}

/// Store for tracking multiple aircraft
pub struct AircraftStore {
    aircraft: HashMap<u32, Aircraft>,
    ttl: Duration,
    /// Minimum messages required before aircraft is considered confirmed
    min_messages: u64,
}

impl AircraftStore {
    #[allow(dead_code)]
    pub fn new(ttl_secs: u64) -> Self {
        Self::with_min_messages(ttl_secs, 2)
    }

    /// Create a new store with custom minimum message threshold
    pub fn with_min_messages(ttl_secs: u64, min_messages: u64) -> Self {
        Self {
            aircraft: HashMap::new(),
            ttl: Duration::from_secs(ttl_secs),
            min_messages,
        }
    }

    /// Update aircraft from a decoded message
    pub fn update_from_message(&mut self, mm: &ModesMessage) -> Option<&Aircraft> {
        let addr = mm.icao_address();

        let aircraft = self
            .aircraft
            .entry(addr)
            .or_insert_with(|| Aircraft::new(addr));
        aircraft.seen = Instant::now();
        aircraft.messages += 1;

        // Track signal quality
        if mm.signal_level > 0 {
            // Running average of signal level
            if aircraft.signal_level == 0 {
                aircraft.signal_level = mm.signal_level;
            } else {
                aircraft.signal_level = ((aircraft.signal_level as u32 * 7 + mm.signal_level as u32) / 8) as u16;
            }
        }
        if mm.phase_corrected {
            aircraft.phase_corrections += 1;
        }

        match mm.msg_type {
            0 | 4 | 16 | 20 => {
                aircraft.altitude = mm.altitude;

                // Extract BDS data if present (DF20)
                if mm.msg_type == 20 {
                    if let Some(ref bds) = mm.bds_data {
                        self.update_from_bds(addr, bds);
                    }
                }
            }
            5 | 21 => {
                // Store squawk (identity) code
                if mm.identity != 0 {
                    aircraft.squawk = mm.identity;
                }
                
                // Extract BDS data if present (DF21)
                if mm.msg_type == 21 {
                    if let Some(ref bds) = mm.bds_data {
                        self.update_from_bds(addr, bds);
                    }
                }
            }
            17 => {
                if (1..=4).contains(&mm.me_type) {
                    aircraft.flight = mm.flight.clone();
                } else if (9..=18).contains(&mm.me_type) {
                    aircraft.altitude = mm.altitude;

                    if mm.fflag {
                        aircraft.odd_cprlat = mm.raw_latitude;
                        aircraft.odd_cprlon = mm.raw_longitude;
                        aircraft.odd_cprtime = Instant::now();
                    } else {
                        aircraft.even_cprlat = mm.raw_latitude;
                        aircraft.even_cprlon = mm.raw_longitude;
                        aircraft.even_cprtime = Instant::now();
                    }

                    let time_diff = if aircraft.even_cprtime > aircraft.odd_cprtime {
                        aircraft.even_cprtime.duration_since(aircraft.odd_cprtime)
                    } else {
                        aircraft.odd_cprtime.duration_since(aircraft.even_cprtime)
                    };

                    if time_diff <= Duration::from_secs(10) {
                        self.decode_cpr(addr);
                    }
                } else if mm.me_type == 19 {
                    if mm.me_sub == 1 || mm.me_sub == 2 {
                        aircraft.speed = mm.velocity;
                        aircraft.track = mm.heading as u16;
                    }
                }
            }
            _ => {}
        }

        self.aircraft.get(&addr)
    }

    /// Update aircraft with BDS data
    fn update_from_bds(&mut self, addr: u32, bds: &BdsData) {
        let aircraft = match self.aircraft.get_mut(&addr) {
            Some(a) => a,
            None => return,
        };

        match bds {
            BdsData::AircraftIdentification { callsign } => {
                if aircraft.flight.is_empty() {
                    aircraft.flight = callsign.clone();
                }
            }
            BdsData::SelectedVerticalIntention {
                mcp_altitude,
                baro_setting,
                ..
            } => {
                if let Some(alt) = mcp_altitude {
                    aircraft.selected_altitude = Some(*alt);
                }
                if let Some(baro) = baro_setting {
                    aircraft.baro_setting = Some(*baro);
                }
            }
            BdsData::TrackAndTurnReport {
                roll_angle,
                ground_speed,
                true_airspeed,
                true_track,
                ..
            } => {
                if let Some(roll) = roll_angle {
                    aircraft.roll_angle = Some(*roll);
                }
                if let Some(gs) = ground_speed {
                    aircraft.speed = *gs;
                }
                if let Some(tas) = true_airspeed {
                    aircraft.true_airspeed = Some(*tas);
                }
                if let Some(track) = true_track {
                    aircraft.track = *track as u16;
                }
            }
            BdsData::HeadingAndSpeedReport {
                magnetic_heading,
                indicated_airspeed,
                mach,
                baro_altitude_rate,
                ..
            } => {
                if let Some(hdg) = magnetic_heading {
                    aircraft.magnetic_heading = Some(*hdg);
                }
                if let Some(ias) = indicated_airspeed {
                    aircraft.indicated_airspeed = Some(*ias);
                }
                if let Some(m) = mach {
                    aircraft.mach = Some(*m);
                }
                if let Some(rate) = baro_altitude_rate {
                    aircraft.baro_altitude_rate = Some(*rate);
                }
            }
            _ => {}
        }
    }

    /// Get aircraft by ICAO address
    #[allow(dead_code)]
    pub fn get(&self, addr: u32) -> Option<&Aircraft> {
        self.aircraft.get(&addr)
    }

    /// Get all aircraft that meet the minimum message threshold
    pub fn all(&self) -> impl Iterator<Item = &Aircraft> {
        let min_msg = self.min_messages;
        self.aircraft.values().filter(move |a| a.messages >= min_msg)
    }

    /// Get all aircraft including those below message threshold (for internal use)
    #[allow(dead_code)]
    pub fn all_unfiltered(&self) -> impl Iterator<Item = &Aircraft> {
        self.aircraft.values()
    }

    /// Remove stale aircraft
    pub fn remove_stale(&mut self) {
        let now = Instant::now();
        self.aircraft
            .retain(|_, a| now.duration_since(a.seen) <= self.ttl);
    }

    /// Number of tracked aircraft (meeting minimum message threshold)
    #[allow(dead_code)]
    pub fn len(&self) -> usize {
        self.all().count()
    }

    /// Number of all tracked aircraft including below threshold
    #[allow(dead_code)]
    pub fn len_total(&self) -> usize {
        self.aircraft.len()
    }

    #[allow(dead_code)]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Decode CPR coordinates for an aircraft.
    fn decode_cpr(&mut self, addr: u32) {
        let aircraft = match self.aircraft.get_mut(&addr) {
            Some(a) => a,
            None => return,
        };

        const AIR_DLAT0: f64 = 360.0 / 60.0;
        const AIR_DLAT1: f64 = 360.0 / 59.0;

        let lat0 = aircraft.even_cprlat as f64;
        let lat1 = aircraft.odd_cprlat as f64;
        let lon0 = aircraft.even_cprlon as f64;
        let lon1 = aircraft.odd_cprlon as f64;

        let j = ((59.0 * lat0 - 60.0 * lat1) / 131072.0 + 0.5).floor() as i32;

        let mut rlat0 = AIR_DLAT0 * (cpr_mod(j, 60) as f64 + lat0 / 131072.0);
        let mut rlat1 = AIR_DLAT1 * (cpr_mod(j, 59) as f64 + lat1 / 131072.0);

        if rlat0 >= 270.0 {
            rlat0 -= 360.0;
        }
        if rlat1 >= 270.0 {
            rlat1 -= 360.0;
        }

        if cpr_nl(rlat0) != cpr_nl(rlat1) {
            return;
        }

        if aircraft.even_cprtime > aircraft.odd_cprtime {
            let ni = cpr_n(rlat0, false);
            let m = ((lon0 * (cpr_nl(rlat0) - 1) as f64 - lon1 * cpr_nl(rlat0) as f64) / 131072.0
                + 0.5)
                .floor() as i32;
            aircraft.lon = cpr_dlon(rlat0, false) * (cpr_mod(m, ni) as f64 + lon0 / 131072.0);
            aircraft.lat = rlat0;
        } else {
            let ni = cpr_n(rlat1, true);
            let m = ((lon0 * (cpr_nl(rlat1) - 1) as f64 - lon1 * cpr_nl(rlat1) as f64) / 131072.0
                + 0.5)
                .floor() as i32;
            aircraft.lon = cpr_dlon(rlat1, true) * (cpr_mod(m, ni) as f64 + lon1 / 131072.0);
            aircraft.lat = rlat1;
        }

        if aircraft.lon > 180.0 {
            aircraft.lon -= 360.0;
        }
    }

    /// Generate JSON representation of all aircraft
    #[allow(dead_code)]
    pub fn to_json(&self) -> String {
        let mut json = String::from("[\n");
        let mut first = true;

        for aircraft in self.aircraft.values() {
            if aircraft.lat == 0.0 && aircraft.lon == 0.0 {
                continue;
            }

            if !first {
                json.push_str(",\n");
            }
            first = false;

            // Build extended JSON with BDS data
            let mut extra = String::new();

            if let Some(ias) = aircraft.indicated_airspeed {
                extra.push_str(&format!(r#","ias":{}"#, ias));
            }
            if let Some(tas) = aircraft.true_airspeed {
                extra.push_str(&format!(r#","tas":{}"#, tas));
            }
            if let Some(mach) = aircraft.mach {
                extra.push_str(&format!(r#","mach":{:.3}"#, mach));
            }
            if let Some(roll) = aircraft.roll_angle {
                extra.push_str(&format!(r#","roll":{:.1}"#, roll));
            }
            if let Some(hdg) = aircraft.magnetic_heading {
                extra.push_str(&format!(r#","mag_hdg":{:.1}"#, hdg));
            }
            if let Some(rate) = aircraft.baro_altitude_rate {
                extra.push_str(&format!(r#","vert_rate":{}"#, rate));
            }
            if let Some(sel_alt) = aircraft.selected_altitude {
                extra.push_str(&format!(r#","sel_alt":{}"#, sel_alt));
            }
            if let Some(baro) = aircraft.baro_setting {
                extra.push_str(&format!(r#","baro":{:.1}"#, baro));
            }

            json.push_str(&format!(
                r#"{{"hex":"{}","flight":"{}","lat": {},"lon":{},"altitude": {},"track":{},"speed":{}{}}}"#,
                aircraft. hex_addr,
                aircraft.flight,
                aircraft.lat,
                aircraft. lon,
                aircraft.altitude,
                aircraft.track,
                aircraft.speed,
                extra
            ));
        }

        json.push_str("\n]");
        json
    }
}

/// CPR modulo function (always positive)
fn cpr_mod(a: i32, b: i32) -> i32 {
    let res = a % b;
    if res < 0 { res + b } else { res }
}

/// CPR NL function - number of longitude zones at given latitude
fn cpr_nl(lat: f64) -> i32 {
    let lat = lat.abs();

    if lat < 10.47047130 {
        59
    } else if lat < 14.82817437 {
        58
    } else if lat < 18.18626357 {
        57
    } else if lat < 21.02939493 {
        56
    } else if lat < 23.54504487 {
        55
    } else if lat < 25.82924707 {
        54
    } else if lat < 27.93898710 {
        53
    } else if lat < 29.91135686 {
        52
    } else if lat < 31.77209708 {
        51
    } else if lat < 33.53993436 {
        50
    } else if lat < 35.22899598 {
        49
    } else if lat < 36.85025108 {
        48
    } else if lat < 38.41241892 {
        47
    } else if lat < 39.92256684 {
        46
    } else if lat < 41.38651832 {
        45
    } else if lat < 42.80914012 {
        44
    } else if lat < 44.19454951 {
        43
    } else if lat < 45.54626723 {
        42
    } else if lat < 46.86733252 {
        41
    } else if lat < 48.16039128 {
        40
    } else if lat < 49.42776439 {
        39
    } else if lat < 50.67150166 {
        38
    } else if lat < 51.89342469 {
        37
    } else if lat < 53.09516153 {
        36
    } else if lat < 54.27817472 {
        35
    } else if lat < 55.44378444 {
        34
    } else if lat < 56.59318756 {
        33
    } else if lat < 57.72747354 {
        32
    } else if lat < 58.84763776 {
        31
    } else if lat < 59.95459277 {
        30
    } else if lat < 61.04917774 {
        29
    } else if lat < 62.13216659 {
        28
    } else if lat < 63.20427479 {
        27
    } else if lat < 64.26616523 {
        26
    } else if lat < 65.31845310 {
        25
    } else if lat < 66.36171008 {
        24
    } else if lat < 67.39646774 {
        23
    } else if lat < 68.42322022 {
        22
    } else if lat < 69.44242631 {
        21
    } else if lat < 70.45451075 {
        20
    } else if lat < 71.45986473 {
        19
    } else if lat < 72.45884545 {
        18
    } else if lat < 73.45177442 {
        17
    } else if lat < 74.43893416 {
        16
    } else if lat < 75.42056257 {
        15
    } else if lat < 76.39684391 {
        14
    } else if lat < 77.36789461 {
        13
    } else if lat < 78.33374083 {
        12
    } else if lat < 79.29428225 {
        11
    } else if lat < 80.24923213 {
        10
    } else if lat < 81.19801349 {
        9
    } else if lat < 82.13956981 {
        8
    } else if lat < 83.07199445 {
        7
    } else if lat < 83.99173563 {
        6
    } else if lat < 84.89166191 {
        5
    } else if lat < 85.75541621 {
        4
    } else if lat < 86.53536998 {
        3
    } else if lat < 87.00000000 {
        2
    } else {
        1
    }
}

/// CPR N function
fn cpr_n(lat: f64, is_odd: bool) -> i32 {
    let nl = cpr_nl(lat) - if is_odd { 1 } else { 0 };
    if nl < 1 { 1 } else { nl }
}

/// CPR Dlon function
fn cpr_dlon(lat: f64, is_odd: bool) -> f64 {
    360.0 / cpr_n(lat, is_odd) as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cpr_nl() {
        assert_eq!(cpr_nl(0.0), 59);
        assert_eq!(cpr_nl(45.0), 42);
        assert_eq!(cpr_nl(89.0), 1);
    }

    #[test]
    fn test_cpr_mod() {
        assert_eq!(cpr_mod(5, 3), 2);
        assert_eq!(cpr_mod(-1, 3), 2);
        assert_eq!(cpr_mod(-5, 3), 1);
    }

    #[test]
    fn test_aircraft_new() {
        let ac = Aircraft::new(0x4840D6);
        assert_eq!(ac.addr, 0x4840D6);
        assert_eq!(ac.hex_addr, "4840D6");
        assert_eq!(ac.messages, 0);
        assert!(ac.roll_angle.is_none());
        assert!(ac.mach.is_none());
    }

    #[test]
    fn test_aircraft_store() {
        let store = AircraftStore::new(60);
        assert!(store.is_empty());
        assert_eq!(store.len(), 0);
    }
}
