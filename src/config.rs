//! Configuration and command-line argument parsing

use std::env;

#[derive(Debug, Clone)]
pub struct Config {
    // Device settings
    pub dev_index: u32,
    pub gain: i32,
    pub enable_agc: bool,
    pub freq: u32,

    // Input
    pub filename: Option<String>,
    pub loop_file: bool,

    // Processing
    pub fix_errors: bool,
    pub check_crc: bool,
    pub aggressive: bool,

    // Output
    pub raw: bool,
    pub onlyaddr: bool,
    pub metric: bool,
    pub interactive: bool,
    pub interactive_rows: usize,
    pub interactive_ttl: u64,
    /// Minimum messages required before showing aircraft (filter ghosts)
    pub min_messages: u64,

    // Receiver position (for distance/bearing calculation)
    /// Receiver latitude (optional)
    pub receiver_lat: Option<f64>,
    /// Receiver longitude (optional)
    pub receiver_lon: Option<f64>,

    // Networking
    pub net: bool,
    pub net_only: bool,
    pub net_ro_port: u16,
    pub net_ri_port: u16,
    pub net_http_port: u16,
    pub net_sbs_port: u16,

    // Debug
    pub debug: DebugFlags,
    pub stats: bool,
}

#[derive(Debug, Clone, Default)]
pub struct DebugFlags {
    pub demod: bool,
    pub demod_err: bool,
    pub bad_crc: bool,
    pub good_crc: bool,
    pub no_preamble: bool,
    pub net: bool,
    pub js: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            dev_index: 0,
            gain: 999999, // Max gain
            enable_agc: false,
            freq: 1_090_000_000,
            filename: None,
            loop_file: false,
            fix_errors: true,
            check_crc: true,
            aggressive: false,
            raw: false,
            onlyaddr: false,
            metric: true,
            interactive: false,
            interactive_rows: 15,
            interactive_ttl: 60,
            min_messages: 2,
            receiver_lat: None,
            receiver_lon: None,
            net: false,
            net_only: false,
            net_ro_port: 30002,
            net_ri_port: 30001,
            net_http_port: 8080,
            net_sbs_port: 30003,
            debug: DebugFlags::default(),
            stats: false,
        }
    }
}

impl Config {
    pub fn from_args() -> Self {
        let args: Vec<String> = env::args().collect();
        let mut config = Config::default();

        let mut i = 1;
        while i < args.len() {
            match args[i].as_str() {
                "--device-index" => {
                    i += 1;
                    config.dev_index = args.get(i).and_then(|s| s.parse().ok()).unwrap_or(0);
                }
                "--gain" => {
                    i += 1;
                    config.gain = args
                        .get(i)
                        .and_then(|s| s.parse::<f64>().ok())
                        .map(|g| (g * 10.0) as i32)
                        .unwrap_or(999999);
                }
                "--enable-agc" => config.enable_agc = true,
                "--freq" => {
                    i += 1;
                    config.freq = args
                        .get(i)
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(1_090_000_000);
                }
                "--ifile" => {
                    i += 1;
                    config.filename = args.get(i).cloned();
                }
                "--loop" => config.loop_file = true,
                "--no-fix" => config.fix_errors = false,
                "--no-crc-check" => config.check_crc = false,
                "--raw" => config.raw = true,
                "--net" => config.net = true,
                "--net-only" => {
                    config.net = true;
                    config.net_only = true;
                }
                "--net-ro-port" => {
                    i += 1;
                    config.net_ro_port = args.get(i).and_then(|s| s.parse().ok()).unwrap_or(30002);
                }
                "--net-ri-port" => {
                    i += 1;
                    config.net_ri_port = args.get(i).and_then(|s| s.parse().ok()).unwrap_or(30001);
                }
                "--net-http-port" => {
                    i += 1;
                    config.net_http_port = args.get(i).and_then(|s| s.parse().ok()).unwrap_or(8080);
                }
                "--net-sbs-port" => {
                    i += 1;
                    config.net_sbs_port = args.get(i).and_then(|s| s.parse().ok()).unwrap_or(30003);
                }
                "--onlyaddr" => config.onlyaddr = true,
                "--metric" => config.metric = true,
                "--imperial" => config.metric = false,
                "--aggressive" => config.aggressive = true,
                "--interactive" => config.interactive = true,
                "--interactive-rows" => {
                    i += 1;
                    config.interactive_rows =
                        args.get(i).and_then(|s| s.parse().ok()).unwrap_or(15);
                }
                "--interactive-ttl" => {
                    i += 1;
                    config.interactive_ttl = args.get(i).and_then(|s| s.parse().ok()).unwrap_or(60);
                }
                "--min-messages" => {
                    i += 1;
                    config.min_messages = args.get(i).and_then(|s| s.parse().ok()).unwrap_or(2);
                }
                "--lat" => {
                    i += 1;
                    config.receiver_lat = args.get(i).and_then(|s| s.parse().ok());
                }
                "--lon" => {
                    i += 1;
                    config.receiver_lon = args.get(i).and_then(|s| s.parse().ok());
                }
                "--stats" => config.stats = true,
                "--debug" => {
                    i += 1;
                    if let Some(flags) = args.get(i) {
                        for c in flags.chars() {
                            match c {
                                'D' => config.debug.demod = true,
                                'd' => config.debug.demod_err = true,
                                'C' => config.debug.good_crc = true,
                                'c' => config.debug.bad_crc = true,
                                'p' => config.debug.no_preamble = true,
                                'n' => config.debug.net = true,
                                'j' => config.debug.js = true,
                                _ => {}
                            }
                        }
                    }
                }
                "--help" => {
                    print_help();
                    std::process::exit(0);
                }
                _ => {
                    eprintln!("Unknown option: {}", args[i]);
                    print_help();
                    std::process::exit(1);
                }
            }
            i += 1;
        }

        config
    }
}

fn print_help() {
    println!(
        r#"dump1090-rs - Mode S decoder for RTL-SDR devices

Usage: dump1090-rs [OPTIONS]

Options:
  --device-index <N>     Select RTL device (default: 0)
  --gain <db>            Set gain (default: max. Use -10 for auto-gain)
  --enable-agc           Enable Automatic Gain Control
  --freq <hz>            Set frequency (default: 1090 MHz)
  --ifile <filename>     Read data from file (use '-' for stdin)
  --loop                 With --ifile, read the same file in a loop
  --interactive          Interactive mode refreshing data on screen
  --interactive-rows <N> Max rows in interactive mode (default: 15)
  --interactive-ttl <s>  Remove from list if idle for <s> seconds (default: 60)
  --raw                  Show only messages hex values
  --net                  Enable networking
  --net-only             Enable just networking, no RTL device or file
  --net-ro-port <port>   TCP port for raw output (default: 30002)
  --net-ri-port <port>   TCP port for raw input (default: 30001)
  --net-http-port <port> HTTP server port (default: 8080)
  --net-sbs-port <port>  TCP port for SBS output (default: 30003)
  --no-fix               Disable single-bit error correction
  --no-crc-check         Disable CRC check (discouraged)
  --aggressive           More CPU for more messages
  --stats                With --ifile print stats at exit
  --onlyaddr             Show only ICAO addresses
  --metric               Use metric units
  --min-messages <N>     Min messages before showing aircraft (default: 2)
  --lat <degrees>        Receiver latitude for distance calculation
  --lon <degrees>        Receiver longitude for distance calculation
  --debug <flags>        Debug mode (d/D/c/C/p/n/j)
  --help                 Show this help
"#
    );
}
