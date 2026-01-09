//! dump1090-rs:  A Mode S decoder for RTL-SDR devices
//!
//! Rust port of antirez/dump1090
//! 

#![allow(dead_code)]

mod aircraft;
mod config;
mod crc;
mod decoder;
mod demodulator;
mod magnitude;
mod network;

use std::io::{self, Write};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crossbeam_channel::{bounded, Receiver, Sender};
use parking_lot:: RwLock;
use tracing::{error, info, Level};
use tracing_subscriber:: FmtSubscriber;

use crate::aircraft:: AircraftStore;
use crate::config::Config;
use crate::decoder::ModesMessage;
use crate:: demodulator::Demodulator;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config = Config::from_args();

    // Initialize logging only if not in interactive mode
    if !config.interactive {
        let subscriber = FmtSubscriber:: builder()
            .with_max_level(Level::INFO)
            .finish();
        tracing::subscriber::set_global_default(subscriber).ok();
        info!("dump1090-rs starting...");
                info!("Configuration: {:?}", config);
    }

    // Shared aircraft store
    let aircraft_store = Arc:: new(RwLock::new(AircraftStore::new(config.interactive_ttl)));

    // Channel for decoded messages
    let (msg_tx, msg_rx): (Sender<ModesMessage>, Receiver<ModesMessage>) = bounded(1024);

    // Start the runtime
    let rt = tokio::runtime::Runtime:: new()?;

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

        // Interactive display task
        let interactive_handle = if config.interactive {
            let store = Arc::clone(&aircraft_store);
            let rows = config.interactive_rows;
            let metric = config.metric;
            Some(tokio::spawn(async move {
                interactive_display(store, rows, metric).await;
            }))
        } else {
            None
        };

        // Data acquisition and demodulation
        if ! config.net_only {
            run_demodulation(&config, msg_tx).await;
        } else {
            if config.interactive {
                // In interactive + net-only mode, just wait
                loop {
                    tokio::time::sleep(Duration::from_secs(1)).await;
                }
            } else {
                info!("Net-only mode, waiting for data from network clients");
                tokio::signal::ctrl_c().await.ok();
            }
        }

        // Cleanup
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
    let demodulator = Demodulator::new(config. clone());

    if let Some(ref filename) = config.filename {
        if ! config.interactive {
            info!("Reading from file: {}", filename);
        }
        if let Err(e) = demodulator.process_file(filename, &msg_tx) {
            error!("Error processing file: {}", e);
        }
    } else {
        // Try to use rtl_sdr command
        if ! config.interactive {
            info!("Attempting to read from RTL-SDR using rtl_sdr command.. .");
        }
        if let Err(e) = run_rtlsdr_command(config, &msg_tx).await {
            error!("Error with RTL-SDR: {}", e);
            if !config.interactive {
                eprintln!("\nMake sure rtl-sdr is installed:  sudo dnf install rtl-sdr");
                eprintln!("Or use --ifile to read from a file, or --net-only for network mode");
            }
        }
    }
}

async fn run_rtlsdr_command(
    config: &Config,
    msg_tx: &Sender<ModesMessage>,
) -> Result<(), Box<dyn std::error::Error>> {
    use std::process:: Stdio;
    use tokio::io::AsyncReadExt;
    use tokio::process::Command;

    let demodulator = Demodulator:: new(config.clone());

    // Build rtl_sdr command
    let mut cmd = Command::new("rtl_sdr");
    cmd.arg("-f")
        .arg(config.freq. to_string())
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
            match stdout.read(&mut data[overlap + total_read..overlap + read_size]).await {
                Ok(0) => return Ok(()), // EOF
                Ok(n) => total_read += n,
                Err(e) => return Err(e.into()),
            }
        }

        // Process the data
        let magnitude =
            crate::magnitude::compute_magnitude_vector(&data[..overlap + read_size], &demodulator.mag_lut);
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
                println!("{}", msg.to_raw_string());
            } else if config.onlyaddr {
                println!("{:06X}", msg.icao_address());
            } else {
                println!("{}", msg);
            }
        }
    }
}

async fn interactive_display(store: Arc<RwLock<AircraftStore>>, max_rows: usize, metric:  bool) {
    let mut last_update = Instant::now();
    let refresh_interval = Duration::from_millis(250);

    loop {
        tokio::time::sleep(Duration::from_millis(50)).await;

        if last_update.elapsed() >= refresh_interval {
            last_update = Instant::now();

            // Clear screen and move cursor to top
            print!("\x1B[2J\x1B[H");

            // Print header
            println!(
                "\x1B[1m{: <6} {: <8} {: >9} {:>7} {:>10} {:>11} {:>5} {:>9} {:>6}\x1B[0m",
                "Hex", "Flight", "Altitude", "Speed", "Lat", "Lon", "Track", "Messages", "Seen"
            );
            println!("{}", "-".repeat(80));

            // Get aircraft data
            let store = store.read();
            let now = Instant::now();

            let mut aircraft:  Vec<_> = store.all().collect();
            // Sort by most recently seen
            aircraft.sort_by(|a, b| b.seen.cmp(&a.seen));

            let mut count = 0;
            for ac in aircraft.iter().take(max_rows) {
                let seen_secs = now.duration_since(ac.seen).as_secs();

                let (altitude, speed) = if metric {
                    (
                        (ac.altitude as f64 / 3.2808) as i32,
                        (ac.speed as f64 * 1.852) as u16,
                    )
                } else {
                    (ac. altitude, ac.speed)
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

                let lat_str = if ac.lat != 0.0 {
                    format! ("{:.4}", ac.lat)
                } else {
                    String:: new()
                };

                let lon_str = if ac.lon != 0.0 {
                    format!("{:.4}", ac.lon)
                } else {
                    String::new()
                };

                let track_str = if ac.track != 0 {
                    format! ("{}", ac.track)
                } else {
                    String:: new()
                };

                println!(
                    "{: <6} {:<8} {: >9} {:>7} {: >10} {:>11} {: >5} {:>9} {: >4}s",
                    ac.hex_addr,
                    ac.flight,
                    alt_str,
                    speed_str,
                    lat_str,
                    lon_str,
                    track_str,
                    ac.messages,
                    seen_secs
                );

                count += 1;
            }

            // Print footer
            println! ("{}", "-".repeat(80));
            println!(
                "Aircraft:  {} | {} mode | Press Ctrl+C to exit",
                count,
                if metric { "Metric" } else { "Imperial" }
            );

            io::stdout().flush().ok();
        }
    }
}