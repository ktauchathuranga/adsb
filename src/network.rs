//! Network services for dump1090-rs
//!  
//!  Provides TCP servers for:
//! - Raw output (port 30002): Sends decoded messages in hex format
//! - Raw input (port 30001): Receives hex messages for decoding
//! - SBS/BaseStation output (port 30003): Aircraft data in SBS format
//! - HTTP server (port 8080): Web interface with aircraft map

use std::sync::Arc;
use parking_lot::RwLock;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::broadcast;
use tracing::{info, error, debug};

use crate::aircraft::AircraftStore;
use crate::config::Config;
use crate:: decoder;

/// Message broadcast channel capacity
const BROADCAST_CAPACITY: usize = 1024;

/// Run all network servers
pub async fn run_servers(
    config: Config,
    aircraft_store: Arc<RwLock<AircraftStore>>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    // Create broadcast channels for distributing messages to clients
        let (raw_tx, _) = broadcast::channel:: <String>(BROADCAST_CAPACITY);
    let (sbs_tx, _) = broadcast::channel::<String>(BROADCAST_CAPACITY);
    
    // Spawn server tasks
    let raw_out_handle = {
        let tx = raw_tx.clone();
        let port = config.net_ro_port;
        tokio::spawn(async move {
            if let Err(e) = run_raw_output_server(port, tx).await {
                error!("Raw output server error: {}", e);
            }
        })
    };
    
    let raw_in_handle = {
        let port = config.net_ri_port;
        let store = Arc::clone(&aircraft_store);
        let cfg = config.clone();
        let tx = raw_tx.clone();
        tokio::spawn(async move {
            if let Err(e) = run_raw_input_server(port, store, cfg, tx).await {
                error!("Raw input server error:  {}", e);
            }
        })
    };
    
    let sbs_handle = {
        let tx = sbs_tx.clone();
        let port = config.net_sbs_port;
        tokio::spawn(async move {
            if let Err(e) = run_sbs_server(port, tx).await {
                error!("SBS server error: {}", e);
            }
        })
    };
    
    let http_handle = {
        let port = config.net_http_port;
        let store = Arc::clone(&aircraft_store);
        tokio::spawn(async move {
            if let Err(e) = run_http_server(port, store).await {
                error!("HTTP server error: {}", e);
            }
        })
    };
    
    // Wait for all servers (they run forever unless error)
    tokio::select! {
        _ = raw_out_handle => {}
        _ = raw_in_handle => {}
        _ = sbs_handle => {}
        _ = http_handle => {}
    }
    
    Ok(())
}

/// Raw output server (port 30002)
/// Broadcasts decoded messages in *HEX; format
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
                    Err(broadcast::error::RecvError:: Lagged(_)) => continue,
                    Err(_) => break,
                }
            }
            debug!("Raw output client disconnected:  {}", addr);
        });
    }
}

/// Raw input server (port 30001)
/// Accepts hex messages and decodes them
async fn run_raw_input_server(
    port: u16,
    store: Arc<RwLock<AircraftStore>>,
    config: Config,
    broadcast_tx: broadcast::Sender<String>,
) -> Result<(), Box<dyn std::error:: Error + Send + Sync>> {
    let listener = TcpListener::bind(format!("0.0.0.0:{}", port)).await?;
    info!("Raw input server listening on port {}", port);
    
    loop {
        let (socket, addr) = listener.accept().await?;
        debug!("Raw input client connected: {}", addr);
        
        let store = Arc::clone(&store);
        let config = config.clone();
        let tx = broadcast_tx.clone();
        
        tokio::spawn(async move {
            handle_raw_input_client(socket, store, config, tx).await;
            debug!("Raw input client disconnected: {}", addr);
        });
    }
}

async fn handle_raw_input_client(
    socket: TcpStream,
    store: Arc<RwLock<AircraftStore>>,
    config: Config,
    tx: broadcast:: Sender<String>,
) {
    let reader = BufReader::new(socket);
    let mut lines = reader.lines();
    
    while let Ok(Some(line)) = lines.next_line().await {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        
        if let Some(mm) = decoder::decode_hex_message(line, config.fix_errors, config.aggressive) {
            if mm.crc_ok || ! config.check_crc {
                // Update aircraft store
                {
                    let mut store = store.write();
                    store.update_from_message(&mm);
                }
                
                // Broadcast to raw output clients
                let _ = tx.send(mm.to_raw_string());
            }
        }
    }
}

/// SBS/BaseStation output server (port 30003)
async fn run_sbs_server(
    port: u16,
    tx: broadcast::Sender<String>,
) -> Result<(), Box<dyn std::error:: Error + Send + Sync>> {
    let listener = TcpListener::bind(format!("0.0.0.0:{}", port)).await?;
    info!("SBS server listening on port {}", port);
    
    loop {
        let (socket, addr) = listener.accept().await?;
        debug!("SBS client connected: {}", addr);
        
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
                    Err(broadcast::error::RecvError:: Lagged(_)) => continue,
                    Err(_) => break,
                }
            }
            debug!("SBS client disconnected: {}", addr);
        });
    }
}

/// HTTP server (port 8080)
async fn run_http_server(
    port: u16,
    store: Arc<RwLock<AircraftStore>>,
) -> Result<(), Box<dyn std::error:: Error + Send + Sync>> {
    let listener = TcpListener::bind(format!("0.0.0.0:{}", port)).await?;
    info!("HTTP server listening on port {}", port);
    
    loop {
        let (socket, addr) = listener.accept().await?;
        debug!("HTTP client connected: {}", addr);
        
        let store = Arc::clone(&store);
        
        tokio::spawn(async move {
            if let Err(e) = handle_http_client(socket, store).await {
                debug!("HTTP client error: {}", e);
            }
        });
    }
}

async fn handle_http_client(
    mut socket: TcpStream,
    store: Arc<RwLock<AircraftStore>>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let mut buf = vec![0u8; 4096];
    let n = socket.peek(&mut buf).await?;
    
    if n == 0 {
        return Ok(());
    }
    
    let request = String::from_utf8_lossy(&buf[..n]);
    
    // Parse HTTP request (minimal parsing)
    let path = request
        .lines()
        .next()
        .and_then(|line| line.split_whitespace().nth(1))
        .unwrap_or("/");
    
    // Drain the request from the socket
    let reader = BufReader::new(&mut socket);
    let mut lines = reader.lines();
    while let Ok(Some(line)) = lines.next_line().await {
        if line.is_empty() {
            break;
        }
    }
    
    let (content_type, content) = if path == "/data. json" {
        let store = store.read();
        ("application/json", store.to_json())
    } else {
        ("text/html", get_map_html().to_string())
    };
    
    let response = format!(
        "HTTP/1.1 200 OK\r\n\
         Content-Type: {}\r\n\
         Content-Length:  {}\r\n\
         Access-Control-Allow-Origin: *\r\n\
         Connection: close\r\n\
         \r\n{}",
        content_type,
        content.len(),
        content
    );
    
    socket.write_all(response.as_bytes()).await?;
    
    Ok(())
}

/// HTML for the map interface
fn get_map_html() -> &'static str {
    r#"<! DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="initial-scale=1.0, user-scalable=no" />
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet. css" />
    <style>
        html, body { height: 100%; margin: 0; padding: 0; }
        #map { height: 100%; width: 80%; float: left; }
        #info { width: 20%; height: 100%; float: right; background: #fff; 
                border-left: 1px solid #666; font-family:  Helvetica, sans-serif; 
                padding:  10px; box-sizing: border-box; overflow-y: auto; }
        #info h1 { font-size: 18px; margin-top: 0; }
        #info p { font-size: 14px; color: #333; }
        . plane-icon { font-size: 20px; }
    </style>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
</head>
<body>
    <div id="map"></div>
    <div id="info">
        <h1>dump1090-rs</h1>
        <p id="count">Loading...</p>
        <p id="selected">Click on a plane for info. </p>
    </div>
    <script>
        const map = L.map('map').setView([40.0, -100.0], 4);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(map);
        
        const planes = {};
        let selected = null;
        
        function updatePlanes() {
            fetch('/data.json')
                .then(r => r.json())
                .then(data => {
                    const seen = {};
                    data.forEach(p => {
                        seen[p.hex] = true;
                        if (planes[p.hex]) {
                            planes[p.hex].setLatLng([p.lat, p.lon]);
                            planes[p.hex].data = p;
                        } else {
                            const icon = L.divIcon({
                                html: '<div style="transform: rotate(' + (45-p.track) + 'deg)">✈️</div>',
                                className: 'plane-icon'
                            });
                            const marker = L.marker([p.lat, p.lon], {icon}).addTo(map);
                            marker.data = p;
                            marker.on('click', () => {
                                selected = p. hex;
                                updateSelected();
                            });
                            planes[p.hex] = marker;
                        }
                    });
                    
                    // Remove old planes
                    Object.keys(planes).forEach(hex => {
                        if (!seen[hex]) {
                            map.removeLayer(planes[hex]);
                            delete planes[hex];
                        }
                    });
                    
                    document.getElementById('count').textContent = 
                        Object.keys(planes).length + ' aircraft tracked';
                    updateSelected();
                })
                .catch(e => console.error(e));
        }
        
        function updateSelected() {
            if (selected && planes[selected]) {
                const p = planes[selected]. data;
                document.getElementById('selected').innerHTML = 
                    '<b>' + p.hex + '</b><br>' +
                    'Flight: ' + (p.flight || 'N/A') + '<br>' +
                    'Altitude: ' + p.altitude + ' ft<br>' +
                    'Speed: ' + p.speed + ' kt<br>' +
                    'Track: ' + p.track + '°<br>' +
                    'Position: ' + p.lat. toFixed(4) + ', ' + p.lon.toFixed(4);
            }
        }
        
        setInterval(updatePlanes, 1000);
        updatePlanes();
    </script>
</body>
</html>"#
}