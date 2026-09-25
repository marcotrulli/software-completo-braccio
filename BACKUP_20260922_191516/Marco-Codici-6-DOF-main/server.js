const express = require('express');
const { SerialPort } = require('serialport');
const cors = require('cors');
const path = require('path');
const fs = require('fs');
const http = require('http');
const { spawn } = require('child_process');

const app = express();
const PORT = 8765;
const KINECT_PORT = 8766;

app.use(cors());
app.use(express.json({ limit: '50mb' }));
app.use(express.static(__dirname));

const CONFIG_DIR = path.join(__dirname, 'config');

function saveConfigFile(fileName, data) {
    fs.mkdirSync(CONFIG_DIR, { recursive: true });
    const target = path.join(CONFIG_DIR, fileName);
    const tmp = target + '.tmp';
    fs.writeFileSync(tmp, JSON.stringify(data, null, 2));
    fs.renameSync(tmp, target);
}

// === TINKERBOARD COMPAT: anche formati C++ ===
function loadTinkerConfigs() {
    // Espone anche i file TinkerBoard se presenti, per compatibilità
    const files = ['robot6dof_poses.json','robot6dof_animations.json','robot6dof_physics.json','robot6dof_speeds.json','robot6dof_notes.txt'];
    const out = {};
    for (const f of files) {
        try { out[f] = fs.readFileSync(path.join(CONFIG_DIR, f), 'utf8'); } catch(e) { out[f] = null; }
    }
    return out;
}

// === SERIALE ESP32 - ROBUSTA + AUTO-DETECT + JSON PROTOCOL ===
let serialPort = null;
let esp32Connected = false;
let esp32State = { connected: false, armed: false, q: [0,0,0,0,0,0], port: null, lastSeen: 0, error: null };
let serialLogs = [];
let serialBuffer = '';
let pendingCmd = null; // {id, type, sentMs, timeoutMs}
const ESP32_BAUDRATE = 115200;
const SERIAL_LOG_MAX = 120;
const PREFERRED_COM = 'COM5';
const USB_SERIAL_VIDS = ['1A86','10C4','0403','303A','2341','2E8A','1B4F','2A03','03EB'];
let serialReconnectTimer = null;
let lastFailKey = '';
let lastFailAt = 0;
let heartbeatStarted = false;
let noStateCount = 0;
let lastRescanAt = 0;
let connectInFlight = null;   // single-flight: max 1 connect alla volta
let portGeneration = 0;       // invalida handler della porta chiusa

function pushLog(line) {
    const ts = new Date().toISOString().substr(11,8);
    serialLogs.push(`[${ts}] ${line}`);
    if (serialLogs.length > SERIAL_LOG_MAX) serialLogs.shift();
    console.log(line);
}

function parseJsonLine(line) {
    if (!line.startsWith('{')) return null;
    try { return JSON.parse(line); } catch(e) { return null; }
}

function handleSerialLine(raw) {
    const line = raw.trim();
    if (!line) return;
    pushLog(`< ${line}`);
    const obj = parseJsonLine(line);
    if (!obj) return;
    if (obj.device !== 'robot6dof' || obj.protocol !== 1) return;
    esp32State.lastSeen = Date.now();
    esp32State.connected = true;
    esp32Connected = true;
    if (obj.type === 'state') {
        if (Array.isArray(obj.q) && obj.q.length === 6) esp32State.q = obj.q.map(v=>+v);
        esp32State.armed = !!obj.armed;
        esp32State.error = null;
    } else if (obj.type === 'done' || obj.type === 'stopped' || obj.type === 'armed' || obj.type === 'pong') {
        if (pendingCmd && pendingCmd.id.toUpperCase() === String(obj.id).toUpperCase()) {
            pushLog(`✓ ${obj.type} ${obj.id}`);
            pendingCmd = null;
            if (obj.type === 'armed') esp32State.armed = true;
        } else if (pendingCmd) {
            // fallback: se id non matcha per case, sblocca comunque dopo done/armed (firmware uppercase)
            if (['done','armed'].includes(obj.type)) pendingCmd = null;
        }
    } else if (obj.type === 'cal_step' || obj.type === 'cal_done') {
        // CAL: il firmware manda cal_step/finché cal_done non arriva
        pushLog(`🧭 CAL ${obj.type}: step=${obj.cal_step}/${obj.cal_steps} done=${obj.cal_done}`);
        esp32State.calStep = obj.cal_step;
        esp32State.calSteps = obj.cal_steps;
        esp32State.calDone = obj.cal_done;
        if (obj.type === 'cal_done' && pendingCmd && pendingCmd.type === 'CAL') {
            pushLog(`✓ CAL completata ${obj.id}`);
            pendingCmd = null;
        }
    } else if (obj.type === 'error') {
        esp32State.error = obj.error || 'errore firmware';
        pushLog(`✗ ERROR ${obj.id}: ${esp32State.error}`);
        if (pendingCmd && pendingCmd.id.toUpperCase() === String(obj.id).toUpperCase()) pendingCmd = null;
    }
}

function isBluetoothPort(p) {
    const s = `${p.path||''} ${p.manufacturer||''} ${p.pnpId||''}`;
    return /BTHENUM|Bluetooth/i.test(s);
}

function isMotherboardPort(p) {
    const s = `${p.path||''} ${p.manufacturer||''} ${p.pnpId||''} ${p.friendlyId||''}`;
    if ((p.path||'').toUpperCase() === 'COM1') return true;
    return /PNP0501|Tipi di porte standard|Standard serial|motherboard/i.test(s);
}

function isLikelyEsp32Port(p) {
    if (isBluetoothPort(p) || isMotherboardPort(p)) return false;
    const vid = String(p.vendorId||'').toUpperCase();
    if (USB_SERIAL_VIDS.includes(vid)) return true;
    const path = String(p.path||'').toUpperCase();
    if (path.includes('USB') || path.includes('ACM')) return true;
    return false;
}

function scheduleReconnect(reason, delayMs) {
    if (serialReconnectTimer) return;
    serialReconnectTimer = setTimeout(() => {
        serialReconnectTimer = null;
        if (!esp32Connected && !connectInFlight) {
            connectToESP32(null, true).catch(()=>{});
        }
    }, delayMs);
}

function logFailOnce(key, msg) {
    const now = Date.now();
    if (key === lastFailKey && now - lastFailAt < 15000) {
        lastFailAt = now;
        return;
    }
    lastFailKey = key;
    lastFailAt = now;
    pushLog(msg);
}

// Cerca porta: COM5 se esiste, altrimenti USB-serial reale (mai COM1/Bluetooth)
async function findEsp32Port() {
    const ports = await SerialPort.list();
    console.log('Porte seriali disponibili:', ports.map(p => `${p.path} (${p.manufacturer||''} ${p.productId||''})`));
    if (!ports.length) return null;
    const preferred = ports.find(p => (p.path||'').toUpperCase() === PREFERRED_COM);
    if (preferred) {
        console.log(`→ Uso prioritario ${PREFERRED_COM}`);
        return preferred;
    }
    const usb = ports.find(p => isLikelyEsp32Port(p));
    if (usb) {
        console.log(`→ Uso porta USB-serial ${usb.path}`);
        return usb;
    }
    console.log('Nessuna porta ESP32 (USB-serial) disponibile');
    return null;
}

function startHeartbeat() {
    if (heartbeatStarted) return;
    heartbeatStarted = true;
    setInterval(() => {
        if (esp32Connected && esp32State.lastSeen !== 0 && Date.now() - esp32State.lastSeen > 4000) {
            noStateCount++;
            if (noStateCount === 1) {
                pushLog('⚠️ Timeout ESP32 - nessun state da 4s');
            }
            if (noStateCount > 6) {
                pushLog('🔄 Nessun state, riconnessione automatica...');
                noStateCount = 0;
                if (serialPort) try{ serialPort.close(); }catch(e){}
                scheduleReconnect('no-state', 1000);
            }
        } else {
            noStateCount = 0;
        }
        if (!esp32Connected && !serialReconnectTimer && !connectInFlight) {
            const now = Date.now();
            if (now - lastRescanAt > 5000) {
                lastRescanAt = now;
                connectToESP32(null, true).catch(()=>{});
            }
        }
        if (pendingCmd && Date.now() - pendingCmd.sentMs > pendingCmd.timeoutMs) {
            pushLog(`⏱ Timeout comando ${pendingCmd.id}`);
            pendingCmd = null;
        }
    }, 1000);
}

async function connectToESP32(preferredPath = null, quiet = false) {
    // Single-flight: se un connect è già in corso, ritorna quello (niente doppie aperture => niente Access denied)
    if (connectInFlight) return connectInFlight;
    connectInFlight = _connectToESP32Inner(preferredPath, quiet)
        .finally(() => { connectInFlight = null; });
    return connectInFlight;
}

async function _connectToESP32Inner(preferredPath = null, quiet = false) {
    startHeartbeat();
    try {
        // Chiudi porta precedente e stacca i suoi handler (generation token)
        portGeneration++;
        const myGen = portGeneration;
        if (serialPort) {
            const old = serialPort;
            serialPort = null;
            try { old.removeAllListeners('data'); old.removeAllListeners('error'); old.removeAllListeners('close'); } catch(e){}
            if (old.isOpen) {
                await new Promise(r => { try { old.close(() => r()); } catch(e) { r(); } });
                // Attesa rilascio handle Windows (evita Access denied se riapriamo subito)
                await new Promise(r => setTimeout(r, 400));
            }
        }
        esp32Connected = false;
        esp32State.connected = false;

        let targetPort = null;
        const ports = await SerialPort.list();
        if (preferredPath) {
            targetPort = ports.find(p => p.path === preferredPath) || null;
            if (!targetPort) {
                if (!quiet) logFailOnce(`missing:${preferredPath}`, `⚠️ Porta ${preferredPath} non disponibile, auto-detect...`);
                targetPort = await findEsp32Port();
            } else if (!quiet) {
                console.log(`Tentativo connessione a ${preferredPath}`);
            }
        } else {
            targetPort = await findEsp32Port();
        }

        if (!targetPort) {
            if (!quiet) {
                console.log('Nessuna porta ESP32 trovata, modalità solo simulazione');
                pushLog('⚠️ Nessuna porta trovata - simulazione');
            }
            return false;
        }

        // Se un altro connect è partito nel frattempo, abbandona
        if (myGen !== portGeneration) return false;

        const port = new SerialPort({
            path: targetPort.path,
            baudRate: ESP32_BAUDRATE,
            autoOpen: false
        });

        await new Promise((resolve, reject) => {
            port.open((err) => {
                if (err) {
                    console.error('Errore apertura porta seriale:', err.message);
                    logFailOnce(`open:${targetPort.path}:${err.message}`, `✗ Errore apertura ${targetPort.path}: ${err.message}`);
                    esp32Connected = false;
                    // Access denied: riprova con backoff (handle Windows ancora occupato)
                    if (/Access denied|in use|Cannot open/i.test(err.message)) {
                        scheduleReconnect('access-denied', 2500);
                    }
                    reject(err);
                } else {
                    console.log(`✅ Connesso a ESP32 su ${targetPort.path}`);
                    pushLog(`✅ Connesso a ESP32 su ${targetPort.path} @${ESP32_BAUDRATE}`);
                    lastFailKey = '';
                    esp32Connected = true;
                    esp32State.connected = true;
                    esp32State.port = targetPort.path;
                    esp32State.lastSeen = Date.now();
                    noStateCount = 0;
                    resolve();
                }
            });
        });

        // Connessione annullata da un connect più nuovo?
        if (myGen !== portGeneration) {
            try { port.close(); } catch(e){}
            return false;
        }

        serialPort = port;
        serialBuffer = '';
        port.on('data', (chunk) => {
            if (myGen !== portGeneration) return;
            serialBuffer += chunk.toString('utf8');
            let idx;
            while ((idx = serialBuffer.indexOf('\n')) !== -1) {
                const line = serialBuffer.slice(0, idx);
                serialBuffer = serialBuffer.slice(idx + 1);
                handleSerialLine(line);
            }
            if (serialBuffer.length > 8192) serialBuffer = '';
        });

        port.on('error', (err) => {
            if (myGen !== portGeneration) return;
            console.error('Errore seriale:', err.message);
            logFailOnce(`err:${err.message}`, `✗ Errore seriale: ${err.message}`);
            esp32Connected = false;
            esp32State.connected = false;
            esp32State.error = err.message;
        });

        port.on('close', () => {
            if (myGen !== portGeneration) return; // chiusura intenzionale di un'apertura vecchia
            console.log('Connessione seriale chiusa');
            logFailOnce('close', '⚠️ Connessione seriale chiusa - riconnessione automatica...');
            esp32Connected = false;
            esp32State.connected = false;
            serialPort = null;
            scheduleReconnect('close', 2000);
        });

        return true;
    } catch (error) {
        console.error('Errore connessione ESP32:', error.message);
        if (!quiet) pushLog(`✗ Errore connessione: ${error.message}`);
        return false;
    }
}

function sendRawCommand(command, timeoutMs = 4000) {
    return new Promise((resolve, reject) => {
        if (!esp32Connected || !serialPort || !serialPort.isOpen) {
            return reject(new Error('ESP32 non connesso'));
        }
        // Se c'è un comando pending, rifiuta (come TinkerBoard)
        if (pendingCmd) {
            return reject(new Error('Comando già in corso: ' + pendingCmd.id));
        }
        const idMatch = command.match(/^(SEG|ARM|STOP|PING|CAL)\s+(\S+)/);
        const id = idMatch ? idMatch[2] : `cmd_${Date.now()}`;
        const type = idMatch ? idMatch[1] : 'RAW';
        pendingCmd = { id, type, sentMs: Date.now(), timeoutMs };
        const line = command + '\n';
        pushLog(`> ${command}`);
        serialPort.write(line, (err) => {
            if (err) {
                pendingCmd = null;
                return reject(err);
            }
            resolve({ id, type });
        });
    });
}

// === KINECT BRIDGE (Kinect for Windows SDK 1.8 -> porta 8766) ===
let kinectChild = null;

function startKinectBridge() {
    const exe = path.join(__dirname, 'kinect', 'KinectBridge.exe');
    if (!fs.existsSync(exe)) {
        console.log('⚠️ kinect/KinectBridge.exe non trovato - Modalità 2 (Kinect) non disponibile');
        return;
    }
    try {
        kinectChild = spawn(exe, [], {
            cwd: path.dirname(exe),
            stdio: ['ignore', 'pipe', 'pipe'],
            windowsHide: true
        });
        const kinLog = (d) => String(d).split(/\r?\n/).filter(Boolean).forEach(l => console.log('[kinect] ' + l));
        if (kinectChild.stdout) kinectChild.stdout.on('data', kinLog);
        if (kinectChild.stderr) kinectChild.stderr.on('data', kinLog);
        kinectChild.on('error', (e) => {
            console.log('⚠️ Errore Kinect bridge: ' + e.message);
            kinectChild = null;
        });
        kinectChild.on('exit', (code) => {
            console.log('Kinect bridge uscito (code=' + code + ')');
            kinectChild = null;
        });
        console.log('📷 Kinect bridge avviato -> http://127.0.0.1:' + KINECT_PORT);
    } catch (e) {
        console.log('⚠️ Kinect bridge non avviabile: ' + e.message);
    }
}

function stopKinectBridge() {
    if (kinectChild) {
        try { kinectChild.kill(); } catch (e) {}
        kinectChild = null;
    }
}

process.on('exit', stopKinectBridge);
process.on('SIGINT', () => { stopKinectBridge(); process.exit(0); });
process.on('SIGTERM', () => { stopKinectBridge(); process.exit(0); });

// Stato Kinect (proxy del bridge su 8766) - la UI puo usarlo come fallback
app.get('/api/kinect/status', (req, res) => {
    const reqHttp = http.get('http://127.0.0.1:' + KINECT_PORT + '/status', { timeout: 1500 }, (r) => {
        let data = '';
        r.on('data', (c) => { data += c; });
        r.on('end', () => {
            try { res.json(JSON.parse(data)); }
            catch (e) { res.json({ ok: false, running: false, error: 'Risposta bridge non valida' }); }
        });
    });
    reqHttp.on('error', (e) => {
        res.json({ ok: false, running: false, error: 'Kinect bridge non attivo (' + e.code + ')' });
    });
    reqHttp.on('timeout', () => {
        reqHttp.destroy();
        res.json({ ok: false, running: false, error: 'Kinect bridge timeout' });
    });
});

// === API ===
app.get('/api/esp32/status', (req, res) => {
    res.json({
        connected: esp32Connected && esp32State.connected,
        armed: esp32State.armed,
        q: esp32State.q,
        port: serialPort ? serialPort.path : esp32State.port,
        error: esp32State.error,
        lastSeen: esp32State.lastSeen
    });
});

// Compat TinkerBoard: stesso endpoint ma con stato esteso
app.get('/api/tinker/status', (req, res) => {
    res.json({
        connected: esp32Connected,
        armed: esp32State.armed,
        q: esp32State.q,
        port: esp32State.port,
        calStep: esp32State.calStep,
        calSteps: esp32State.calSteps,
        calDone: esp32State.calDone,
        pending: pendingCmd,
        logs: serialLogs.slice(-20)
    });
});

app.post('/api/esp32/send', async (req, res) => {
    const { command } = req.body;
    if (!command) return res.status(400).json({ error: 'Comando mancante' });
    if (!esp32Connected || !serialPort || !serialPort.isOpen) {
            return res.status(503).json({ error: 'ESP32 non connesso', message: 'Collega ESP32 via USB e premi Riconnetti' });
    }
    try {
        const info = await sendRawCommand(command);
        res.json({ success: true, command, id: info.id });
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});

// Nuovo: invio strutturato Tinker (SEG/ARM) con validazione
app.post('/api/tinker/seg', async (req, res) => {
    const { id, duration_ms, q } = req.body;
    if (!Array.isArray(q) || q.length !== 6) return res.status(400).json({ error: 'q deve essere array[6]' });
    const cmd = `SEG ${id||'seg_'+Date.now()} ${duration_ms||1000} ${q.map(v=>Number(v).toFixed(2)).join(' ')}`;
    try {
        const info = await sendRawCommand(cmd, (duration_ms||1000)+4000);
        res.json({ success: true, command: cmd, id: info.id });
    } catch(e) { res.status(500).json({ error: e.message }); }
});
app.post('/api/tinker/arm', async (req, res) => {
    const { q } = req.body;
    if (!Array.isArray(q) || q.length !== 6) return res.status(400).json({ error: 'q deve essere array[6]' });
    const cmd = `ARM arm ${q.map(v=>Number(v).toFixed(2)).join(' ')}`;
    try { const info = await sendRawCommand(cmd, 2500); res.json({ success:true, command:cmd, id:info.id }); } catch(e){ res.status(500).json({error:e.message}); }
});
app.post('/api/tinker/cal', async (req, res) => {
    const id = (req.body && req.body.id) ? req.body.id : 'cal_' + Date.now();
    const cmd = `CAL ${id}`;
    try { const info = await sendRawCommand(cmd, 150000); res.json({ success:true, command:cmd, id:info.id }); } catch(e){ res.status(500).json({error:e.message}); }
});

app.get('/api/serial/ports', async (req, res) => {
    try {
        const ports = await SerialPort.list();
        // Evidenzia porta preferita / USB-serial
        const enriched = ports.map(p => ({
            path: p.path,
            manufacturer: p.manufacturer || '',
            productId: p.productId || '',
            vendorId: p.vendorId || '',
            isCom5: p.path.toUpperCase() === PREFERRED_COM
        }));
        res.json({ ports: enriched.map(e=>e.path), details: enriched, com5Available: enriched.some(e=>e.isCom5) });
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});

app.post('/api/esp32/reconnect', async (req, res) => {
    const { port } = req.body || {};
    const target = port || null;
    // Se un connect è già in corso, aspetta quello invece di aprirne un altro
    if (connectInFlight) {
        try { await connectInFlight; } catch(e){}
    } else if (serialPort) {
        portGeneration++;
        try { serialPort.removeAllListeners('data'); serialPort.removeAllListeners('error'); serialPort.removeAllListeners('close'); } catch(e){}
        try {
            if (serialPort.isOpen) {
                await new Promise(r => serialPort.close(err => r()));
                await new Promise(r => setTimeout(r, 900));
            }
        } catch(e){}
        serialPort = null;
        esp32Connected = false;
        esp32State.connected = false;
    }
    let success = await connectToESP32(target, true);
    if (!success) {
        success = await connectToESP32(null, false);
    }
    res.json({ success, connected: esp32Connected, port: esp32State.port, error: esp32State.error });
});

// Alias per compatibilità
app.post('/api/tinker/reconnect', async (req,res)=>{
    const { port } = req.body || {};
    const success = await connectToESP32(port || null, true);
    res.json({ success, connected: esp32Connected, port: esp32State.port });
});

app.get('/api/esp32/logs', (req, res) => {
    res.json({ logs: serialLogs.slice(-40), connected: esp32Connected && esp32State.connected, state: esp32State, pending: pendingCmd });
});
app.get('/api/tinker/logs', (req,res)=>{
    res.json({ logs: serialLogs, state: esp32State, pending: pendingCmd });
});

// Salva configurazioni - PC + Tinker compat
app.post('/api/save_stl_config', (req, res) => {
    try { saveConfigFile('robot6dof_stl_config.json', req.body); res.json({ status: 'ok' }); } catch (e){ res.status(500).json({ error: e.message }); }
});
app.post('/api/save_poses_config', (req, res) => {
    try {
        saveConfigFile('robot6dof_saved_poses.json', req.body);
        // Compat Tinker: salva anche formato poses_store (version 2)
        try {
            const tinkerPoses = { version: 2, poses: Object.entries(req.body.poses||{}).map(([name,q])=>({name, q})) };
            // aggiunge anche current pose se presente
            saveConfigFile('robot6dof_poses.json', tinkerPoses);
        } catch(e){}
        res.json({ status: 'ok' });
    } catch (e){ res.status(500).json({ error: e.message }); }
});
app.post('/api/save_anim_config', (req, res) => {
    try {
        saveConfigFile('robot6dof_animations.json', req.body);
        // Compat Tinker: converte animationGroups -> groups con wait_s/wait_user/action
        try {
            const src = req.body.animationGroups || req.body.groups || {};
            const groups = Object.entries(src).map(([name, steps])=>({
                name,
                steps: (steps||[]).map(s=>({
                    name: s.name||s.id||'step',
                    q: s.q||[0,0,0,0,0,0],
                    wait_s: s.waitTime||s.wait_s||0,
                    wait_user: (s.waitType==='user_check'||s.wait_user) ? true : false,
                    action: s.action||0,
                    grab_mass_g: s.grab_mass_g||0
                }))
            }));
            saveConfigFile('robot6dof_animations_tinker.json', { version:2, groups });
        } catch(e){}
        res.json({ status: 'ok' });
    } catch (e){ res.status(500).json({ error: e.message }); }
});
// Nuovi: physics / speeds / notes come TinkerBoard
app.post('/api/save_physics_config', (req,res)=>{
    try{ saveConfigFile('robot6dof_physics.json', req.body); res.json({status:'ok'});}catch(e){res.status(500).json({error:e.message});}
});
app.post('/api/save_speeds_config', (req,res)=>{
    try{ saveConfigFile('robot6dof_speeds.json', req.body); res.json({status:'ok'});}catch(e){res.status(500).json({error:e.message});}
});
app.post('/api/save_notes', (req,res)=>{
    try{
        const txt = req.body.text || req.body.notes || '';
        fs.mkdirSync(CONFIG_DIR,{recursive:true});
        fs.writeFileSync(path.join(CONFIG_DIR,'robot6dof_notes.txt'), txt, 'utf8');
        res.json({status:'ok'});
    }catch(e){res.status(500).json({error:e.message});}
});
app.get('/api/list_stls', (req, res) => {
    try {
        const files = fs.readdirSync(path.join(__dirname, 'models')).filter(f => f.endsWith('.stl'));
        res.json(files);
    } catch (e){ res.status(500).json({ error: e.message }); }
});

// Avvio
async function startServer() {
    console.log('🚀 Avvio Robot 6 DOF Control Center (PC) - auto-detect seriale + TinkerBoard compat...');
    startKinectBridge();
    await connectToESP32(null, true);
    if (!esp32Connected) {
        console.log('ESP32 non trovato - modo simulazione (verrò riprovato ogni 5s)');
    }
    app.listen(PORT, '0.0.0.0', () => {
        console.log(`✅ Server attivo su http://localhost:${PORT}`);
        console.log(`🌐 http://<IP-PC>:${PORT}`);
        console.log(`📡 ESP32 Status: ${esp32Connected ? 'CONNESSO su '+(serialPort?serialPort.path:'?') : 'NON CONNESSO (modo simulazione)'}`);
        console.log(`📁 Config dir: ${CONFIG_DIR}`);
    });
}
startServer();
