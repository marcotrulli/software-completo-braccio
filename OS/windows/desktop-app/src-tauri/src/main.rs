#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::process::{Child, Command};
use std::sync::Mutex;
use std::time::Duration;
use tauri::Manager;

static SERVER_CHILD: Mutex<Option<Child>> = Mutex::new(None);

const PORT: u16 = 8765;
// Percorso assoluto OPZIONALE del progetto: lasciare vuoto ("").
// La cartella viene cercata automaticamente (variabile ROBOT6DOF_PROJECT_DIR,
// cartella dell'eseguibile, cartella corrente e cartelle padre), quindi il
// programma funziona su qualsiasi PC/Linux senza modificare il codice.
const LIVE_PROJECT: &str = "";

fn port_open() -> bool {
    TcpStream::connect(("127.0.0.1", PORT)).is_ok()
}

fn has_server(p: &Path) -> bool {
    p.join("server.js").exists()
}

fn find_project_dir() -> Option<PathBuf> {
    // 1) override esplicito via variabile d'ambiente
    if let Ok(p) = std::env::var("ROBOT6DOF_PROJECT_DIR") {
        let p = PathBuf::from(p);
        if has_server(&p) {
            return Some(p);
        }
    }
    // 2) percorso assoluto opzionale (solo se compilato con un valore)
    if !LIVE_PROJECT.is_empty() {
        let live = PathBuf::from(LIVE_PROJECT);
        if has_server(&live) {
            return Some(live);
        }
    }
    // 3) punti di partenza: cartella dell'eseguibile e cartella corrente
    let mut starts: Vec<PathBuf> = Vec::new();
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            starts.push(dir.to_path_buf());
        }
    }
    if let Ok(dir) = std::env::current_dir() {
        starts.push(dir);
    }
    for start in starts {
        // app installata/bundlata: eseguibile/project oppree Resources/project
        for c in [start.join("project"), start.join("Resources/project")] {
            if has_server(&c) {
                return Some(c);
            }
        }
        // risale le cartelle padre: a ogni livello prova anche la
        // sottocartella "Marco-Codici-6-DOF-main" (struttura del repository)
        let mut cur = Some(start.as_path());
        for _ in 0..8 {
            let dir = match cur {
                Some(d) => d,
                None => break,
            };
            for c in [dir.to_path_buf(), dir.join("Marco-Codici-6-DOF-main")] {
                if has_server(&c) {
                    return Some(c);
                }
            }
            cur = dir.parent();
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
