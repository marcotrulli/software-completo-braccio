#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::process::{Child, Command};
use std::sync::Mutex;
use std::time::Duration;
use tauri::Manager;

static SERVER_CHILD: Mutex<Option<Child>> = Mutex::new(None);

const PORT: u16 = 8765;
const LIVE_PROJECT: &str =
    r"C:\Users\Marco Trulli\Desktop\Marco-Codici-6-DOF-main\Marco-Codici-6-DOF-main";

fn port_open() -> bool {
    TcpStream::connect(("127.0.0.1", PORT)).is_ok()
}

fn find_project_dir() -> Option<PathBuf> {
    if let Ok(p) = std::env::var("ROBOT6DOF_PROJECT_DIR") {
        let p = PathBuf::from(p);
        if p.join("server.js").exists() {
            return Some(p);
        }
    }
    let live = PathBuf::from(LIVE_PROJECT);
    if live.join("server.js").exists() {
        return Some(live);
    }
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            let candidates = [dir.join("project"), dir.join("Resources/project")];
            for c in candidates {
                if c.join("server.js").exists() {
                    return Some(c);
                }
            }
        }
    }
    None
}

fn spawn_server(dir: &Path) -> Result<Child, String> {
    let mut cmd = Command::new("node");
    cmd.arg("server.js").current_dir(dir);
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        cmd.creation_flags(0x08000000);
    }
    cmd.spawn().map_err(|e| match e.kind() {
        std::io::ErrorKind::NotFound => {
            "Node.js non trovato. Installalo da https://nodejs.org".to_string()
        }
        _ => format!("Impossibile avviare node: {e}"),
    })
}

fn navigate(handle: &tauri::AppHandle, url: &str) {
    if let Some(win) = handle.get_webview_window("main") {
        if let Ok(u) = url::Url::parse(url) {
            let _ = win.navigate(u);
        }
    }
}

fn boot(handle: tauri::AppHandle) {
    if port_open() {
        navigate(
            &handle,
            &format!("http://localhost:{PORT}/ik_simulator_v30.html"),
        );
        return;
    }
    let dir = match find_project_dir() {
        Some(d) => d,
        None => {
            navigate(&handle, "error.html?e=Cartella%20progetto%20non%20trovata");
            return;
        }
    };
    match spawn_server(&dir) {
        Ok(child) => *SERVER_CHILD.lock().unwrap() = Some(child),
        Err(e) => {
            let enc: String =
                url::form_urlencoded::byte_serialize(e.as_bytes()).collect();
            navigate(&handle, &format!("error.html?e={enc}"));
            return;
        }
    }
    for _ in 0..200 {
        if port_open() {
            navigate(
                &handle,
                &format!("http://localhost:{PORT}/ik_simulator_v30.html"),
            );
            return;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    navigate(&handle, "error.html?e=Timeout%3A%20server%20non%20avviato%20in%2020s");
}

fn kill_server() {
    if let Some(mut child) = SERVER_CHILD.lock().unwrap().take() {
        let _ = child.kill();
        let _ = child.wait();
    }
}

fn main() {
    let app = tauri::Builder::default()
        .setup(|app| {
            let handle = app.handle().clone();
            std::thread::spawn(move || boot(handle));
            Ok(())
        })
        .build(tauri::generate_context!())
        .expect("errore durante l'avvio di Tauri");

    app.run(|_handle, event| {
        if let tauri::RunEvent::Exit = event {
            kill_server();
        }
    });
}
