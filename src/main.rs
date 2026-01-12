//!   dump1090-rs:   A Mode S decoder for RTL-SDR devices
//! 
//!  Rust port of antirez/dump1090
//! 
//! 

mod aircraft;
mod config;
mod crc;
mod decoder;
mod demodulator;
mod magnitude;
mod network;
mod signal;

use std::io::{self, Write};
use std::sync:: Arc;
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender, bounded};
use parking_lot::RwLock;
use tracing::{Level, info, error};
use tracing_subscriber:: FmtSubscriber;

use crate::aircraft::AircraftStore;
use crate::config::Config;
use crate::decoder::ModesMessage;
use crate::demodulator::Demodulator;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config = Config::from_args();

    // Initialize logging only if not in interactive mode
    if !config.interactive {
        let subscriber = FmtSubscriber:: builder()
            .with_max_level(Level::INFO)
            .finish();
        tracing::subscriber::set_global_default(subscriber).ok();
        info!("dump1090-rs starting.. .");
        info!("Configuration: {:?}", config);
    }

    // Shared aircraft store with min_messages filter from config
    let aircraft_store = Arc::new(RwLock::new(AircraftStore::with_min_messages(
        config.interactive_ttl,
        config.min_messages,
    )));

    // Channel for decoded messages
    let (msg_tx, msg_rx): (Sender<ModesMessage>, Receiver<ModesMessage>) = bounded(1024);

    // Start the runtime
    let rt = tokio::runtime::Runtime::new()?;

    rt.block_on(async {
        // Start network services if enabled
        let net_handle = if config.net || config.net_only {
            let store = Arc::clone(&aircraft_store);
            let cfg = config.clone();
            Some(tokio::spawn(async move {
                if let Err(e) = network::run_servers(cfg, store).await {
                    error!("Network error: {}", e);
                }
            }))
        } else {
            None
        };

        // Message processing task
        let store_for_processor = Arc::clone(&aircraft_store);
        let config_for_processor = config.clone();
        let processor_handle = tokio::spawn(async move {
            process_messages(msg_rx, store_for_processor, config_for_processor).await;
        });

        let interactive_handle = if config.interactive {
            let store = Arc::clone(&aircraft_store);
            let rows = config.interactive_rows;
            let metric = config.metric;
            let receiver_lat = config.receiver_lat;
            let receiver_lon = config.receiver_lon;
            Some(tokio::spawn(async move {
                interactive_display(store, rows, metric, receiver_lat, receiver_lon).await;
            }))
        } else {
            None
        };

        // Stale aircraft removal task
        let cleanup_handle = {
            let store = Arc::clone(&aircraft_store);
            tokio::spawn(async move {
                let mut interval = tokio::time::interval(Duration::from_secs(1));
                loop {
                    interval.tick().await;
                    let mut store = store.write();
                    store.remove_stale();
                }
            })
        };

        // Data acquisition and demodulation
        if ! config.net_only {
            run_demodulation(&config, msg_tx).await;
        }

        // After file processing, keep running if interactive or net mode
        if config.interactive {
            // Show final state and wait for Ctrl+C
            println!("\nFile processing complete. Press Ctrl+C to exit.. .");
            tokio::signal::ctrl_c().await.ok();
        } else if config.net_only {
            info!("Net-only mode, waiting for data from network clients");
            tokio::signal::ctrl_c().await.ok();
        }

        // Cleanup
        cleanup_handle.abort();
        if let Some(h) = net_handle {
            h.abort();
        }
        if let Some(h) = interactive_handle {
            h.abort();
        }
        processor_handle.abort();
    });

    Ok(())
}

async fn run_demodulation(config: &Config, msg_tx: Sender<ModesMessage>) {
    use crate::config::DeviceType;
    
    let demodulator = Demodulator::new(config.clone());

    if let Some(ref filename) = config.filename {
        if !config.interactive {
            info!("Reading from file: {}", filename);
        }
        if let Err(e) = demodulator.process_file(filename, &msg_tx) {
            if !config.interactive {
                error!("Error processing file: {}", e);
            }
        }
    } else {
        // Run appropriate SDR command based on device type
        let result = match config.device_type {
            DeviceType::RtlSdr => {
                if !config.interactive {
                    info!("Attempting to read from RTL-SDR...");
                }
                run_rtlsdr_command(config, &msg_tx).await
            }
            DeviceType::HackRf => {
                if !config.interactive {
                    info!("Attempting to read from HackRF One...");
                }
                run_hackrf_command(config, &msg_tx).await
            }
        };

        if let Err(e) = result {
            let device_name = match config.device_type {
                DeviceType::RtlSdr => "RTL-SDR",
                DeviceType::HackRf => "HackRF",
            };
            error!("Error with {}: {}", device_name, e);
            if !config.interactive {
                match config.device_type {
                    DeviceType::RtlSdr => {
                        eprintln!("\nMake sure rtl-sdr is installed: sudo dnf install rtl-sdr");
                    }
                    DeviceType::HackRf => {
                        eprintln!("\nMake sure hackrf is installed: sudo dnf install hackrf");
                    }
                }
                eprintln!("Or use --ifile to read from a file, or --net-only for network mode");
            }
        }
    }
}

async fn run_rtlsdr_command(
    config: &Config,
    msg_tx: &Sender<ModesMessage>,
) -> Result<(), Box<dyn std::error::Error>> {
    use std::process::Stdio;
    use tokio::io::AsyncReadExt;
    use tokio::process::Command;

    let mut demodulator = Demodulator::new(config.clone());

    // Build rtl_sdr command
    let mut cmd = Command::new("rtl_sdr");
    cmd.arg("-f")
        .arg(config.freq.to_string())
        .arg("-s")
        .arg("2000000")
        .arg("-g")
        .arg(if config.gain < 0 {
            "0".to_string()
        } else {
            (config.gain / 10).to_string()
        })
        .arg("-")
        .stdout(Stdio::piped())
        .stderr(Stdio::null());

    let mut child = cmd.spawn()?;
    let mut stdout = child.stdout.take().ok_or("Failed to get stdout")?;

    let buffer_len = 16 * 16384 + (8 + 112 - 1) * 4;
    let mut data = vec![127u8; buffer_len];
    let read_size = 16 * 16384;

    loop {
        let overlap = (8 + 112 - 1) * 4;
        data.copy_within(read_size..read_size + overlap, 0);

        let mut total_read = 0;
        while total_read < read_size {
            match stdout
                .read(&mut data[overlap + total_read..overlap + read_size])
                .await
            {
                Ok(0) => return Ok(()), // EOF
                Ok(n) => total_read += n,
                Err(e) => return Err(e.into()),
            }
        }

        // Process the data
        let magnitude = crate::magnitude::compute_magnitude_vector(
            &data[..overlap + read_size],
            &demodulator.mag_lut,
        );
        demodulator.detect_modes_external(&magnitude, msg_tx);
    }
}

/// Run HackRF One using hackrf_transfer command
async fn run_hackrf_command(
    config: &Config,
    msg_tx: &Sender<ModesMessage>,
) -> Result<(), Box<dyn std::error::Error>> {
    use std::process::Stdio;
    use tokio::io::AsyncReadExt;
    use tokio::process::Command;

    let mut demodulator = Demodulator::new(config.clone());

    // Build hackrf_transfer command
    // -r - : receive to stdout
    // -f : frequency in Hz
    // -s : sample rate (2M for ADS-B)
    // -a : amp enable (0 or 1)
    // -l : LNA gain (0-40 dB)
    // -g : VGA gain (0-62 dB)
    let mut cmd = Command::new("hackrf_transfer");
    cmd.arg("-r")
        .arg("-")  // Output to stdout
        .arg("-f")
        .arg(config.freq.to_string())
        .arg("-s")
        .arg("2000000")
        .arg("-a")
        .arg("1")  // Enable amp
        .arg("-l")
        .arg("32") // LNA gain
        .arg("-g")
        .arg("20") // VGA gain
        .stdout(Stdio::piped())
        .stderr(Stdio::null());

    let mut child = cmd.spawn()?;
    let mut stdout = child.stdout.take().ok_or("Failed to get stdout")?;

    let buffer_len = 16 * 16384 + (8 + 112 - 1) * 4;
    let mut raw_data = vec![0i8; buffer_len];
    let mut data = vec![127u8; buffer_len];
    let read_size = 16 * 16384;

    loop {
        let overlap = (8 + 112 - 1) * 4;
        
        // Copy overlap region
        for i in 0..overlap {
            raw_data[i] = raw_data[read_size + i];
            data[i] = data[read_size + i];
        }

        let mut total_read = 0;
        while total_read < read_size {
            // Read as bytes, then interpret as signed
            let slice = unsafe {
                std::slice::from_raw_parts_mut(
                    raw_data[overlap + total_read..].as_mut_ptr() as *mut u8,
                    read_size - total_read,
                )
            };
            match stdout.read(slice).await {
                Ok(0) => return Ok(()), // EOF
                Ok(n) => total_read += n,
                Err(e) => return Err(e.into()),
            }
        }

        // Convert signed 8-bit (HackRF) to unsigned 8-bit (RTL-SDR format)
        // HackRF: -128 to 127, centered at 0
        // RTL-SDR: 0 to 255, centered at 127
        for i in 0..overlap + read_size {
            data[i] = (raw_data[i] as i16 + 128) as u8;
        }

        // Process the data (now in RTL-SDR format)
        let magnitude = crate::magnitude::compute_magnitude_vector(
            &data[..overlap + read_size],
            &demodulator.mag_lut,
        );
        demodulator.detect_modes_external(&magnitude, msg_tx);
    }
}

async fn process_messages(
    rx: Receiver<ModesMessage>,
    store: Arc<RwLock<AircraftStore>>,
    config: Config,
) {
    while let Ok(msg) = rx.recv() {
        // Update aircraft tracking
        if msg.crc_ok || ! config.check_crc {
            let mut store = store.write();
            store.update_from_message(&msg);
        }

        // Display in non-interactive mode
        if !config.interactive {
            if config.raw {
                println!("{}", msg. to_raw_string());
            } else if config.onlyaddr {
                println!("{:06X}", msg.icao_address());
            } else {
                println!("{}", msg);
            }
        }
    }
}

async fn interactive_display(
    store: Arc<RwLock<AircraftStore>>,
    max_rows: usize,
    metric: bool,
    receiver_lat: Option<f64>,
    receiver_lon: Option<f64>,
) {
    let refresh_interval = Duration::from_millis(250);

    loop {
        tokio::time::sleep(refresh_interval).await;

        // Clear screen and move cursor to top
        print!("\x1B[2J\x1B[H");
        let _ = io::stdout().flush();

        // ANSI color codes
        const RED: &str = "\x1B[91m";
        const YELLOW: &str = "\x1B[93m";
        const GREEN: &str = "\x1B[92m";
        const BOLD: &str = "\x1B[1m";
        const RESET: &str = "\x1B[0m";

        // Print header based on whether we have receiver position
        let has_position = receiver_lat.is_some() && receiver_lon.is_some();
        
        if has_position {
            println!(
                "{BOLD}{:<6} {:<8} {:>7} {:>5} {:>6} {:>5} {:>5} {:>5} {:>4} {:>6} {:>3}{RESET}",
                "Hex", "Flight", "Alt", "Spd", "Dist", "Brg", "VRate", "IAS", "M", "Msgs", "Age"
            );
        } else {
            println!(
                "{BOLD}{:<6} {:<8} {:>7} {:>5} {:>9} {:>10} {:>5} {:>5} {:>5} {:>4} {:>6} {:>3}{RESET}",
                "Hex", "Flight", "Alt", "Spd", "Lat", "Lon", "Track", "VRate", "IAS", "M", "Msgs", "Age"
            );
        }
        println!("{}", "-".repeat(if has_position { 75 } else { 95 }));

        // Get aircraft data
        let store = store.read();
        let now = Instant::now();

        let mut aircraft: Vec<_> = store.all().collect();
        // Sort by most recently seen
        aircraft.sort_by(|a, b| b.seen.cmp(&a.seen));

        let count = aircraft.len();
        for ac in aircraft.iter().take(max_rows) {
            let seen_secs = now.duration_since(ac.seen).as_secs();

            // Check for emergency squawk codes
            let is_emergency = matches!(ac.squawk, 7500 | 7600 | 7700);
            let squawk_color = match ac.squawk {
                7500 => RED,    // Hijack
                7600 => YELLOW, // Radio failure
                7700 => RED,    // Emergency
                _ => "",
            };

            let (altitude, speed) = if metric {
                (
                    (ac.altitude as f64 / 3.2808) as i32,
                    (ac.speed as f64 * 1.852) as u16,
                )
            } else {
                (ac.altitude, ac.speed)
            };

            let alt_str = if altitude != 0 {
                format!("{}", altitude)
            } else {
                String::new()
            };

            let speed_str = if speed != 0 {
                format!("{}", speed)
            } else {
                String::new()
            };

            // Vertical rate indicator with arrow
            let vrate_str = if let Some(rate) = ac.baro_altitude_rate {
                if rate > 100 {
                    format!("{GREEN}▲{:+}{RESET}", rate)
                } else if rate < -100 {
                    format!("{YELLOW}▼{:+}{RESET}", rate)
                } else {
                    format!("{:+}", rate)
                }
            } else {
                String::new()
            };

            // IAS (Indicated Airspeed)
            let ias_str = ac
                .indicated_airspeed
                .map(|v| format!("{}", v))
                .unwrap_or_default();

            // Mach number
            let mach_str = ac
                .mach
                .map(|m| format!("{:.2}", m))
                .unwrap_or_default();

            // Build the line based on whether we have receiver position
            if has_position {
                let (dist_str, brg_str) = if ac.lat != 0.0 && ac.lon != 0.0 {
                    let (dist, brg) = calculate_distance_bearing(
                        receiver_lat.unwrap(),
                        receiver_lon.unwrap(),
                        ac.lat,
                        ac.lon,
                    );
                    let dist_val = if metric { dist } else { dist * 0.539957 }; // km to nm
                    let unit = if metric { "km" } else { "nm" };
                    (format!("{:.1}{}", dist_val, unit), format!("{:.0}°", brg))
                } else {
                    (String::new(), String::new())
                };

                // Color the hex for emergencies
                let hex_display = if is_emergency {
                    format!("{}{}{}", squawk_color, ac.hex_addr, RESET)
                } else {
                    ac.hex_addr.clone()
                };

                println!(
                    "{:<6} {:<8} {:>7} {:>5} {:>6} {:>5} {:>5} {:>5} {:>4} {:>6} {:>2}s",
                    hex_display,
                    ac.flight,
                    alt_str,
                    speed_str,
                    dist_str,
                    brg_str,
                    vrate_str,
                    ias_str,
                    mach_str,
                    ac.messages,
                    seen_secs
                );
            } else {
                let lat_str = if ac.lat != 0.0 {
                    format!("{:.4}", ac.lat)
                } else {
                    String::new()
                };

                let lon_str = if ac.lon != 0.0 {
                    format!("{:.4}", ac.lon)
                } else {
                    String::new()
                };

                let track_str = if ac.track != 0 {
                    format!("{}", ac.track)
                } else {
                    String::new()
                };

                // Color the hex for emergencies
                let hex_display = if is_emergency {
                    format!("{}{}{}", squawk_color, ac.hex_addr, RESET)
                } else {
                    ac.hex_addr.clone()
                };

                println!(
                    "{:<6} {:<8} {:>7} {:>5} {:>9} {:>10} {:>5} {:>5} {:>5} {:>4} {:>6} {:>2}s",
                    hex_display,
                    ac.flight,
                    alt_str,
                    speed_str,
                    lat_str,
                    lon_str,
                    track_str,
                    vrate_str,
                    ias_str,
                    mach_str,
                    ac.messages,
                    seen_secs
                );
            }

            // Show emergency warning on separate line
            if is_emergency {
                let warning = match ac.squawk {
                    7500 => format!("{RED}  ⚠ HIJACK (7500){RESET}"),
                    7600 => format!("{YELLOW}  ⚠ RADIO FAILURE (7600){RESET}"),
                    7700 => format!("{RED}  ⚠ EMERGENCY (7700){RESET}"),
                    _ => String::new(),
                };
                if !warning.is_empty() {
                    println!("{}", warning);
                }
            }
        }

        // Print footer
        println!("{}", "-".repeat(if has_position { 75 } else { 95 }));
        let pos_info = if has_position {
            format!(
                " | Pos: {:.4},{:.4}",
                receiver_lat.unwrap(),
                receiver_lon.unwrap()
            )
        } else {
            String::new()
        };
        println!(
            "Aircraft: {} | {} mode{} | Ctrl+C to exit",
            count,
            if metric { "Metric" } else { "Imperial" },
            pos_info
        );

        io::stdout().flush().ok();
    }
}

/// Calculate distance (km) and bearing (degrees) between two lat/lon points
/// Uses the Haversine formula
fn calculate_distance_bearing(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> (f64, f64) {
    const EARTH_RADIUS_KM: f64 = 6371.0;

    let lat1_rad = lat1.to_radians();
    let lat2_rad = lat2.to_radians();
    let delta_lat = (lat2 - lat1).to_radians();
    let delta_lon = (lon2 - lon1).to_radians();

    // Haversine distance
    let a = (delta_lat / 2.0).sin().powi(2)
        + lat1_rad.cos() * lat2_rad.cos() * (delta_lon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().asin();
    let distance = EARTH_RADIUS_KM * c;

    // Bearing
    let y = delta_lon.sin() * lat2_rad.cos();
    let x = lat1_rad.cos() * lat2_rad.sin() - lat1_rad.sin() * lat2_rad.cos() * delta_lon.cos();
    let bearing_rad = y.atan2(x);
    let bearing = (bearing_rad.to_degrees() + 360.0) % 360.0;

    (distance, bearing)
}
