//! Network services for dump1090-rs
//!
//!  Mirrors the original dump1090 networking approach. 

use std::fs;
use std::sync::Arc;

use parking_lot::RwLock;
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::net::{TcpListener, TcpStream};
use tokio:: sync::broadcast;
use tracing::{debug, error, info};

use crate:: aircraft::AircraftStore;
use crate::config::Config;
use crate::decoder;

const BROADCAST_CAPACITY: usize = 1024;

pub async fn run_servers(
    config: Config,
    aircraft_store: Arc<RwLock<AircraftStore>>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let (raw_tx, _) = broadcast::channel::<String>(BROADCAST_CAPACITY);
    let (sbs_tx, _) = broadcast::channel::<String>(BROADCAST_CAPACITY);

    let raw_out_handle = {
        let tx = raw_tx.clone();
        let port = config.net_ro_port;
        tokio::spawn(async move {
            if let Err(e) = run_raw_output_server(port, tx).await {
                error!("Raw output server error:  {}", e);
            }
        })
    };

    let raw_in_handle = {
        let port = config.net_ri_port;
        let store = Arc::clone(&aircraft_store);
        let cfg = config. clone();
        let tx = raw_tx.clone();
        tokio::spawn(async move {
            if let Err(e) = run_raw_input_server(port, store, cfg, tx).await {
                error!("Raw input server error: {}", e);
            }
        })
    };

    let sbs_handle = {
        let tx = sbs_tx.clone();
        let port = config.net_sbs_port;
        tokio::spawn(async move {
            if let Err(e) = run_sbs_server(port, tx).await {
                error!("SBS server error:  {}", e);
            }
        })
    };

    let http_handle = {
        let port = config.net_http_port;
        let store = Arc:: clone(&aircraft_store);
        tokio::spawn(async move {
            if let Err(e) = run_http_server(port, store).await {
                error!("HTTP server error: {}", e);
            }
        })
    };

    tokio:: select! {
        _ = raw_out_handle => {}
        _ = raw_in_handle => {}
        _ = sbs_handle => {}
        _ = http_handle => {}
    }

    Ok(())
}

async fn run_raw_output_server(
    port: u16,
    tx: broadcast::Sender<String>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let listener = TcpListener::bind(format!("0.0.0.0:{}", port)).await?;
    info!("Raw output server listening on port {}", port);

    loop {
        let (socket, addr) = listener.accept().await?;
        debug!("Raw output client connected:  {}", addr);
        let mut rx = tx.subscribe();

        tokio::spawn(async move {
            let mut socket = socket;
            loop {
                match rx.recv().await {
                    Ok(msg) => {
                        if socket.write_all(msg.as_bytes()).await.is_err() {
                            break;
                        }
                        if socket.write_all(b"\n").await.is_err() {
                            break;
                        }
                    }
                    Err(broadcast:: error::RecvError::Lagged(_)) => continue,
                    Err(_) => break,
                }
            }
            debug!("Raw output client disconnected: {}", addr);
        });
    }
}

async fn run_raw_input_server(
    port: u16,
    store: Arc<RwLock<AircraftStore>>,
    config: Config,
    broadcast_tx: broadcast::Sender<String>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let listener = TcpListener::bind(format!("0.0.0.0:{}", port)).await?;
    info!("Raw input server listening on port {}", port);

    loop {
        let (socket, addr) = listener.accept().await?;
        debug!("Raw input client connected: {}", addr);

        let store = Arc::clone(&store);
        let config = config.clone();
        let tx = broadcast_tx.clone();

        tokio::spawn(async move {
            let reader = BufReader::new(socket);
            let mut lines = reader.lines();

            while let Ok(Some(line)) = lines.next_line().await {
                let line = line. trim();
                if line.is_empty() {
                    continue;
                }

                if let Some(mm) =
                    decoder::decode_hex_message(line, config.fix_errors, config.aggressive)
                {
                    if mm. crc_ok || ! config.check_crc {
                        {
                            let mut store = store.write();
                            store.update_from_message(&mm);
                        }
                        let _ = tx.send(mm.to_raw_string());
                    }
                }
            }
            debug!("Raw input client disconnected: {}", addr);
        });
    }
}

async fn run_sbs_server(
    port:  u16,
    tx: broadcast::Sender<String>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let listener = TcpListener::bind(format!("0.0.0.0:{}", port)).await?;
    info!("SBS server listening on port {}", port);

    loop {
        let (socket, addr) = listener.accept().await?;
        debug! ("SBS client connected: {}", addr);
        let mut rx = tx.subscribe();

        tokio::spawn(async move {
            let mut socket = socket;
            loop {
                match rx.recv().await {
                    Ok(msg) => {
                        if socket.write_all(msg. as_bytes()).await.is_err() {
                            break;
                        }
                        if socket.write_all(b"\n").await.is_err() {
                            break;
                        }
                    }
                    Err(broadcast::error:: RecvError::Lagged(_)) => continue,
                    Err(_) => break,
                }
            }
            debug! ("SBS client disconnected: {}", addr);
        });
    }
}

async fn run_http_server(
    port: u16,
    store: Arc<RwLock<AircraftStore>>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let listener = TcpListener::bind(format! ("0.0.0.0:{}", port)).await?;
    info!("HTTP server listening on port {}", port);

    loop {
        let (socket, addr) = listener.accept().await?;
        debug!("HTTP client connected: {}", addr);

        let store = Arc::clone(&store);

        tokio::spawn(async move {
            if let Err(e) = handle_http_request(socket, store).await {
                debug!("HTTP error: {}", e);
            }
        });
    }
}

/// Handle HTTP request exactly like original dump1090
async fn handle_http_request(
    mut socket: TcpStream,
    store: Arc<RwLock<AircraftStore>>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let mut buffer = vec![0u8; 8192];
    let n = socket.read(&mut buffer).await?;

    if n == 0 {
        return Ok(());
    }

    let request = String:: from_utf8_lossy(&buffer[..n]);

    // Parse HTTP request
    let first_line = request.lines().next().unwrap_or("");
    let parts: Vec<&str> = first_line.split_whitespace().collect();
    
    if parts.len() < 2 {
        return Ok(());
    }

    let url = parts[1];
    
    // Check HTTP version for keep-alive
    let http_version = if first_line.contains("HTTP/1.1") { 11 } else { 10 };
    
    let keepalive = if http_version == 10 {
        request.to_lowercase().contains("connection: keep-alive")
    } else {
        ! request.to_lowercase().contains("connection: close")
    };

    // Serve content based on URL (matching original dump1090 behavior)
    let (content_type, content) = if url.contains("/data.json") {
        // Return aircraft data as JSON
        let json = aircrafts_to_json(&store);
        ("application/json;charset=utf-8", json)
    } else {
        // Serve gmap. html
        match fs::read_to_string("gmap.html") {
            Ok(html) => ("text/html;charset=utf-8", html),
            Err(e) => {
                let error_msg = format!(
                    "Error opening HTML file: {}",
                    e
                );
                ("text/html;charset=utf-8", error_msg)
            }
        }
    };

    // Build response exactly like original dump1090
    let header = format!(
        "HTTP/1.1 200 OK\r\n\
         Server:  Dump1090\r\n\
         Content-Type: {}\r\n\
         Connection: {}\r\n\
         Content-Length:  {}\r\n\
         Access-Control-Allow-Origin:  *\r\n\
         \r\n",
        content_type,
        if keepalive { "keep-alive" } else { "close" },
        content.len()
    );

    socket.write_all(header.as_bytes()).await?;
    socket.write_all(content.as_bytes()).await?;

    Ok(())
}

/// Generate JSON for aircraft data - matches original dump1090 format exactly
fn aircrafts_to_json(store: &Arc<RwLock<AircraftStore>>) -> String {
    let store = store.read();
    let aircraft:  Vec<_> = store.all().collect();

    if aircraft.is_empty() {
        return "[\n]\n".to_string();
    }

    let mut json = String::from("[\n");

    let mut first = true;
    for ac in aircraft {
        // Only include aircraft with position (matching original behavior)
        if ac.lat != 0.0 && ac.lon != 0.0 {
            if !first {
                json.push_str(",\n");
            }
            first = false;

            json.push_str(&format!(
                "{{\"hex\":\"{}\", \"flight\":\"{}\", \"lat\":{}, \"lon\":{}, \"altitude\":{}, \"track\":{}, \"speed\": {}}}",
                ac.hex_addr,
                ac.flight. trim(),
                ac.lat,
                ac.lon,
                ac.altitude,
                ac.track,
                ac.speed
            ));
        }
    }

    json.push_str("\n]\n");
    json
}