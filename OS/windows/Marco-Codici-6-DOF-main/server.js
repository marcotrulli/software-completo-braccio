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
// Porta seriale prioritaria: sovrascrivibile con ROBOT6DOF_PREFERRED_PORT
// (es. ROBOT6DOF_PREFERRED_PORT=COM3). Se non esiste, il server usa
// automaticamente la prima porta USB-serial collegata all'ESP32.
const PREFERRED_COM = process.env.ROBOT6DOF_PREFERRED_PORT || 'COM5';
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
    const preferred = ports.find(p => (p.path||'').toUpperCase() === PREFERRED_COM.toUpperCase());
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
            isCom5: (p.path||'').toUpperCase() === PREFERRED_COM.toUpperCase()
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

// === VISION / ML DATASET (riconoscimento braccio) ===
const VISION_DIR = path.join(__dirname, 'vision');
const VISION_DS = path.join(VISION_DIR, 'dataset');
const VISION_MODEL_DIR = path.join(VISION_DIR, 'model');

function ensureVisionDirs() {
    fs.mkdirSync(VISION_DS, { recursive: true });
    fs.mkdirSync(VISION_MODEL_DIR, { recursive: true });
}
ensureVisionDirs();

function visionListCaptures() {
    try {
        return fs.readdirSync(VISION_DS)
            .filter(f => f.endsWith('.json'))
            .map(f => {
                try {
                    const meta = JSON.parse(fs.readFileSync(path.join(VISION_DS, f), 'utf8'));
                    return { id: meta.id, file: meta.image, created: meta.created, q: meta.q || null, labeled: !!(meta.joints && meta.joints.length) };
                } catch (e) { return null; }
            })
            .filter(Boolean)
            .sort((a, b) => String(b.created).localeCompare(String(a.created)));
    } catch (e) { return []; }
}

// Scatta/registra foto nel dataset ML
// body: { image: "data:image/jpeg;base64,...", q?: [6], depthB64?: string, note?: string }
app.post('/api/vision/capture', (req, res) => {
    try {
        const { image, q, depthB64, note } = req.body || {};
        if (!image || typeof image !== 'string' || image.indexOf('base64') === -1) {
            return res.status(400).json({ error: 'image dataURL base64 mancante' });
        }
        const comma = image.indexOf(',');
        const b64 = image.slice(comma + 1);
        const buf = Buffer.from(b64, 'base64');
        if (!buf.length) return res.status(400).json({ error: 'immagine vuota' });

        const now = new Date();
        const stamp = now.toISOString().replace(/[:.]/g, '-').slice(0, 19);
        const id = 'cap_' + stamp + '_' + Date.now().toString(36);
        const imgName = id + '.jpg';
        fs.writeFileSync(path.join(VISION_DS, imgName), buf);

        let depthPath = null;
        if (depthB64 && typeof depthB64 === 'string') {
            const dbuf = Buffer.from(depthB64, 'base64');
            if (dbuf.length) {
                depthPath = id + '.depth.bin';
                fs.writeFileSync(path.join(VISION_DS, depthPath), dbuf);
            }
        }

        const meta = {
            id,
            image: imgName,
            depth: depthPath,
            created: now.toISOString(),
            q: Array.isArray(q) ? q.map(v => Number(v)) : null,
            note: note || '',
            joints: [],
            source: 'kinect-capture'
        };
        fs.writeFileSync(path.join(VISION_DS, id + '.json'), JSON.stringify(meta, null, 2));
        pushLog(`📸 Dataset ML: ${imgName}${meta.q ? ' (q salvato)' : ''}`);
        res.json({ success: true, id, image: imgName, count: visionListCaptures().length });
    } catch (e) {
        res.status(500).json({ error: e.message });
    }
});

app.get('/api/vision/dataset', (req, res) => {
    const list = visionListCaptures();
    res.json({ count: list.length, items: list, modelDir: VISION_MODEL_DIR });
});

app.get('/api/vision/image/:id', (req, res) => {
    const id = String(req.params.id).replace(/[^a-zA-Z0-9_\-]/g, '');
    const metaPath = path.join(VISION_DS, id + '.json');
    if (!fs.existsSync(metaPath)) return res.status(404).json({ error: 'non trovato' });
    try {
        const meta = JSON.parse(fs.readFileSync(metaPath, 'utf8'));
        const img = path.join(VISION_DS, meta.image);
        if (!fs.existsSync(img)) return res.status(404).json({ error: 'immagine mancante' });
        res.sendFile(img);
    } catch (e) { res.status(500).json({ error: e.message }); }
});

app.get('/api/vision/depth/:id', (req, res) => {
    const id = String(req.params.id).replace(/[^a-zA-Z0-9_\-]/g, '');
    const metaPath = path.join(VISION_DS, id + '.json');
    if (!fs.existsSync(metaPath)) return res.status(404).json({ error: 'non trovato' });
    try {
        const meta = JSON.parse(fs.readFileSync(metaPath, 'utf8'));
        if (!meta.depth) return res.status(404).json({ error: 'depth non salvata' });
        res.sendFile(path.join(VISION_DS, meta.depth));
    } catch (e) { res.status(500).json({ error: e.message }); }
});

// Salva annotazioni 7 giunti su un capture (per training ML)
// body: { joints: [{x,y,z?,label}] }  // x,y in px 640x480, z mm opzionale
app.post('/api/vision/annotate/:id', (req, res) => {
    const id = String(req.params.id).replace(/[^a-zA-Z0-9_\-]/g, '');
    const metaPath = path.join(VISION_DS, id + '.json');
    if (!fs.existsSync(metaPath)) return res.status(404).json({ error: 'non trovato' });
    try {
        const meta = JSON.parse(fs.readFileSync(metaPath, 'utf8'));
        const joints = Array.isArray(req.body && req.body.joints) ? req.body.joints : [];
        meta.joints = joints.map((j, i) => ({
            label: j.label || ('J' + i),
            x: Number(j.x) || 0,
            y: Number(j.y) || 0,
            z: (j.z === undefined || j.z === null) ? null : Number(j.z)
        }));
        meta.updated = new Date().toISOString();
        fs.writeFileSync(metaPath, JSON.stringify(meta, null, 2));
        res.json({ success: true, id, joints: meta.joints.length });
    } catch (e) { res.status(500).json({ error: e.message }); }
});

// Stato modello ML / IA locale
// In futuro: vision/model/*.onnx caricato qui o nel browser
app.get('/api/vision/model/status', (req, res) => {
    let files = [];
    try { files = fs.readdirSync(VISION_MODEL_DIR); } catch (e) {}
    const hasOnnx = files.some(f => f.toLowerCase().endsWith('.onnx'));
    const hasTflite = files.some(f => f.toLowerCase().endsWith('.tflite'));
    const onnxFile = files.find(f => f.toLowerCase().endsWith('.onnx'));
    let mtime = 0;
    try { mtime = Math.round(fs.statSync(path.join(VISION_MODEL_DIR, onnxFile)).mtimeMs); } catch (e) {}
    res.json({
        ready: hasOnnx || hasTflite,
        backend: hasOnnx ? 'onnx' : (hasTflite ? 'tflite' : 'none'),
        files,
        mtime,
        path: VISION_MODEL_DIR,
        labeled: visionListCaptures().filter(x => x.labeled).length,
        captures: visionListCaptures().length,
        hint: 'Per IA locale: copia modello.keypoints.onnx in vision/model/ e imposta window.RobotVisionLocalAI nel client'
    });
});

// === AUTO-RETRAIN ML ===
// Dopo N nuovi campioni approvati, il server rilancia da solo export+training
// in un processo separato; il browser si aggiorna al nuovo ONNX via mtime.
const ML_DIR = path.join(__dirname, 'vision', 'ml');
const ML_PYTHON = path.join(ML_DIR, '.venv', 'Scripts', 'python.exe');
const AUTO_TRAIN_ENABLED = process.env.VISION_ML_AUTO !== '0';
const AUTO_TRAIN_MIN_NEW = Math.max(5, Number(process.env.VISION_ML_AUTO_MIN || 20));
let mlTrainRunning = false;
let mlLastTrainCount = 0;
let mlLastTrainAt = null;

function visionApprovedCount() {
    return visionListCaptures().filter(x => x.labeled).length;
}

function mlStartTrain() {
    mlTrainRunning = true;
    mlLastTrainAt = new Date().toISOString();
    pushLog(`🧠 Training ML avviato (${visionApprovedCount()} campioni approvati)…`);
    const logFd = fs.openSync(path.join(ML_DIR, 'auto_train.log'), 'a');
    fs.writeSync(logFd, `\n===== train ${mlLastTrainAt} =====\n`);
    const p = spawn(ML_PYTHON, ['auto_train.py'], { cwd: ML_DIR, detached: true, stdio: ['ignore', logFd, logFd] });
    p.unref();
    p.on('exit', code => {
        mlTrainRunning = false;
        fs.closeSync(logFd);
        mlLastTrainCount = visionApprovedCount();
        pushLog(code === 0
            ? '✅ Training ML completato — nuovo modello ONNX attivo'
            : `⚠️ Training ML terminato con errore (exit ${code}) — vedi vision/ml/auto_train.log`);
    });
}

function mlMaybeAutoTrain() {
    if (!AUTO_TRAIN_ENABLED || mlTrainRunning) return;
    if (!fs.existsSync(ML_PYTHON)) return;
    const n = visionApprovedCount();
    if (mlLastTrainCount === 0) { mlLastTrainCount = n; return; } // baseline al primo avvio
    if (n - mlLastTrainCount < AUTO_TRAIN_MIN_NEW) return;
    mlStartTrain();
}

app.get('/api/vision/ml/auto', (req, res) => {
    res.json({
        enabled: AUTO_TRAIN_ENABLED,
        running: mlTrainRunning,
        minNew: AUTO_TRAIN_MIN_NEW,
        approved: visionApprovedCount(),
        lastTrainCount: mlLastTrainCount,
        lastTrainAt: mlLastTrainAt
    });
});

// Retrain manuale immediato (forza anche sotto la soglia)
app.post('/api/vision/ml/retrain', (req, res) => {
    if (!fs.existsSync(ML_PYTHON)) return res.status(503).json({ error: 'venv ML non trovato' });
    if (mlTrainRunning) return res.status(409).json({ error: 'training già in corso' });
    mlStartTrain();
    res.json({ started: true });
});

// === LLM VISIONE LOCALE (LM Studio / Bionic) — rilevamento giunti ===
// Lunghezze 3D medie dei link (BASE-G1, G1-G2) dai campioni approvati.
// Servono al client come vincolo cinematico (link rigidi ± tolleranza).
app.get('/api/vision/links', (req, res) => {
    try {
        const st = { fx: 571.26, fy: 571.26, cx: 320, cy: 240 };
        const l1 = [], l2 = [];
        for (const f of fs.readdirSync(VISION_DS).filter(f => f.endsWith('.json'))) {
            try {
                const meta = JSON.parse(fs.readFileSync(path.join(VISION_DS, f), 'utf8'));
                if (!meta.approved || !Array.isArray(meta.joints) || meta.joints.length !== VISION_JOINTS_N) continue;
                const J = meta.joints;
                if (J.some(j => !j || j.z == null)) continue;
                const P = J.map(j => ({
                    x: (j.x - st.cx) * j.z / st.fx,
                    y: (j.y - st.cy) * j.z / st.fy,
                    z: j.z
                }));
                const d01 = Math.hypot(P[1].x - P[0].x, P[1].y - P[0].y, P[1].z - P[0].z);
                const d12 = Math.hypot(P[2].x - P[1].x, P[2].y - P[1].y, P[2].z - P[1].z);
                if (d01 > 50 && d01 < 2000 && d12 > 50 && d12 < 2000) { l1.push(d01); l2.push(d12); }
            } catch (e) {}
        }
        const avg = a => a.length ? Math.round(a.reduce((x, y) => x + y, 0) / a.length) : null;
        res.json({
            count: l1.length,
            L1: avg(l1),          // BASE -> G1 (mm)
            L2: avg(l2),          // G1 -> G2 (mm)
            tol: 50               // tolleranza default ±50 mm
        });
    } catch (e) { res.status(500).json({ error: e.message }); }
});

const LLM_HOSTS = [
    { host: '127.0.0.1', port: 5000, name: 'LM Studio (lms)' },
    { host: '127.0.0.1', port: 1234, name: 'LM Studio (default)' }
];
// Qwen2.5-VL-7B-Instruct: modello vision migliore per grounding pixel (<=8B).
// Scaricato su D:\LMStudioModels\Qwen2.5-VL-7B-Instruct-GGUF e servito da Bionic (lms, porta 5000).
// Fallback: qwen3-vl-4b, poi gemma-3-4b (gia presenti).
const LLM_MODEL = process.env.VISION_LLM_MODEL || 'qwen2.5-vl-7b-instruct';
const LLM_TIMEOUT_MS = 90000;

function httpJson(method, urlStr, bodyObj, timeoutMs) {
    return new Promise((resolve, reject) => {
        try {
            const u = new URL(urlStr);
            const payload = bodyObj ? JSON.stringify(bodyObj) : null;
            const req = http.request({
                hostname: u.hostname,
                port: u.port || 80,
                path: u.pathname + u.search,
                method,
                headers: payload ? {
                    'Content-Type': 'application/json',
                    'Content-Length': Buffer.byteLength(payload)
                } : {},
                timeout: timeoutMs || 15000
            }, r => {
                const chunks = [];
                r.on('data', c => chunks.push(c));
                r.on('end', () => {
                    const text = Buffer.concat(chunks).toString('utf8');
                    try { resolve({ status: r.statusCode, json: JSON.parse(text), text }); }
                    catch (e) { resolve({ status: r.statusCode, json: null, text }); }
                });
            });
            req.on('timeout', () => { req.destroy(new Error('timeout')); });
            req.on('error', reject);
            if (payload) req.write(payload);
            req.end();
        } catch (e) { reject(e); }
    });
}

async function llmFindAlive() {
    for (const h of LLM_HOSTS) {
        try {
            const r = await httpJson('GET', `http://${h.host}:${h.port}/v1/models`, null, 2500);
            if (r.status === 200 && r.json && Array.isArray(r.json.data)) {
                return { ...h, models: r.json.data.map(m => m.id) };
            }
        } catch (e) {}
    }
    return null;
}

app.get('/api/vision/llm/status', async (req, res) => {
    try {
        const alive = await llmFindAlive();
        if (!alive) {
            return res.json({
                ready: false,
                model: LLM_MODEL,
                hint: 'Avvia LM Studio/Bionic (lms server start) e carica ' + LLM_MODEL
            });
        }
        const hasModel = alive.models.some(id =>
            id === LLM_MODEL || id.endsWith('/' + LLM_MODEL.split('/').pop()) ||
            /qwen2.5-vl/i.test(id) || /qwen3-vl/i.test(id) || /gemma-3/i.test(id)
        );
        res.json({
            ready: hasModel,
            endpoint: `http://${alive.host}:${alive.port}/v1`,
            server: alive.name,
            model: LLM_MODEL,
            models: alive.models
        });
    } catch (e) {
        res.status(500).json({ error: e.message });
    }
});

// Giunti reali del robot: 3 (BASE, G1 fine braccio 1, G2 fine braccio 2).
// La PINZA verrà aggiunta come 4° punto quando progettata.
const VISION_JOINT_LABELS = ['BASE', 'G1', 'G2'];
const VISION_JOINTS_N = VISION_JOINT_LABELS.length;

function parseJointsFromText(text) {
    if (!text) return null;
    const LABELS = VISION_JOINT_LABELS;

    function extractXY(obj, idx) {
        if (!obj || typeof obj !== 'object') return null;
        let x = obj.x, y = obj.y;
        if (x == null || y == null) {
            const bb = obj.bbox_2d || obj.bbox || obj.center || obj.point || obj.xy;
            if (Array.isArray(bb) && bb.length >= 2) {
                if (x == null) x = bb[0];
                if (y == null) y = (bb.length >= 4 && (bb[2] - bb[0]) > 40 && (bb[3] - bb[1]) > 40)
                    ? (bb[0] + bb[2]) / 2 : bb[1] === undefined ? bb[0] : bb[1];
                if (bb.length >= 4 && x != null && y != null && (bb[2] - bb[0]) < 40) {
                    // sembra [x1,y1,x2,y2] stretto → centro
                    x = (bb[0] + bb[2]) / 2;
                    y = (bb[1] + bb[3]) / 2;
                }
            }
        }
        if (x == null || y == null) {
            const nums = [];
            const walk = v => {
                if (typeof v === 'number' && isFinite(v)) nums.push(v);
                else if (Array.isArray(v)) v.forEach(walk);
                else if (v && typeof v === 'object') Object.values(v).forEach(walk);
            };
            walk(obj);
            // prendi primi due numeri plausibili come coordinate
            const px = nums.find(n => n >= 0 && n <= 639);
            const py = nums.filter(n => n >= 0 && n <= 479 && n !== px)[0];
            if (px !== undefined && py !== undefined) { x = px; y = py; }
        }
        const nx = Number(x), ny = Number(y);
        if (!isFinite(nx) || !isFinite(ny)) return null;
        if (nx < -5 || nx > 644 || ny < -5 || ny > 484) return null;
        return {
            label: (typeof obj.label === 'string' && obj.label) || (typeof obj.name === 'string' && obj.name) || LABELS[idx] || ('J' + idx),
            x: Math.round(Math.min(639, Math.max(0, nx))),
            y: Math.round(Math.min(479, Math.max(0, ny))),
            z: (obj.z === undefined || obj.z === null || obj.z === '' || !isFinite(Number(obj.z))) ? null : Number(obj.z)
        };
    }

    // tentativi 1: JSON pulto (object o array)
    const candidates = [];
    let t0 = String(text).trim();
    const fence = t0.match(/```(?:json)?\s*([\s\S]*?)```/);
    if (fence) candidates.push(fence[1].trim());
    candidates.push(t0);
    const a0 = t0.indexOf('['), a1 = t0.lastIndexOf(']');
    if (a0 >= 0 && a1 > a0) candidates.push(t0.slice(a0, a1 + 1));
    const o0 = t0.indexOf('{'), o1 = t0.lastIndexOf('}');
    if (o0 >= 0 && o1 > o0) candidates.push(t0.slice(o0, o1 + 1));

    for (const c of candidates) {
        try {
            const data = JSON.parse(c);
            const joints = Array.isArray(data) ? data : (Array.isArray(data.joints) ? data.joints : null);
            if (joints && joints.length >= VISION_JOINTS_N) {
                const clean = joints.slice(0, VISION_JOINTS_N).map(extractXY);
                if (clean.every(Boolean)) return clean;
            }
            if (joints && joints.length >= VISION_JOINTS_N) {
                // riprendi solo validi e riempi mancanti via extractXY fallback
                const clean = [];
                for (let i = 0; i < VISION_JOINTS_N; i++) {
                    const e = extractXY(joints[i], i);
                    if (e) clean.push(e);
                }
                if (clean.length === VISION_JOINTS_N) return clean;
            }
        } catch (e) {}
    }

    // tentativi 2: regex su coppie x/y anche in JSON rotto
    const re = /"?(?:x|cx|center_x)"?\s*[:=]\s*(-?\d+(?:\.\d+)?)\s*,\s*"?(?:y|cy|center_y)"?\s*[:=]\s*(-?\d+(?:\.\d+)?)/gi;
    const pairs = [];
    let m;
    while ((m = re.exec(t0)) !== null) {
        const x = Number(m[1]), y = Number(m[2]);
        if (isFinite(x) && isFinite(y) && x >= -5 && x <= 644 && y >= -5 && y <= 484) {
            pairs.push({
                label: LABELS[pairs.length] || ('J' + pairs.length),
                x: Math.round(Math.min(639, Math.max(0, x))),
                y: Math.round(Math.min(479, Math.max(0, y))),
                z: null
            });
        }
        if (pairs.length >= VISION_JOINTS_N) break;
    }
    if (pairs.length >= VISION_JOINTS_N) return pairs.slice(0, VISION_JOINTS_N);

    // tentativi 3: array di soli numeri [x,y,x,y,...]
    const nums = (t0.match(/-?\d+(?:\.\d+)?/g) || []).map(Number).filter(isFinite);
    if (nums.length >= VISION_JOINTS_N * 2) {
        const pairs2 = [];
        for (let i = 0; i + 1 < nums.length && pairs2.length < VISION_JOINTS_N; i += 2) {
            const x = nums[i], y = nums[i + 1];
            if (x >= 0 && x <= 639 && y >= 0 && y <= 479) {
                pairs2.push({ label: LABELS[pairs2.length], x: Math.round(x), y: Math.round(y), z: null });
            }
        }
        if (pairs2.length === VISION_JOINTS_N) return pairs2;
    }
    return null;
}

// Analisi frame con LLM visione locale → 7 giunti
// body: { image: dataURL, attempt?: number, feedback?: string }
app.post('/api/vision/llm/analyze', async (req, res) => {
    try {
        const { image, attempt, feedback } = req.body || {};
        if (!image || typeof image !== 'string' || image.indexOf('base64') === -1) {
            return res.status(400).json({ error: 'image dataURL base64 mancante' });
        }
        const alive = await llmFindAlive();
        if (!alive) {
            return res.status(503).json({
                error: 'LLM visione non raggiungibile',
                hint: 'Avvia LM Studio/Bionic: lms server start · modello ' + LLM_MODEL
            });
        }

        const attemptN = Math.max(1, Number(attempt) || 1);
        const imagePart = image.indexOf(',') >= 0 ? image.slice(image.indexOf(',') + 1) : image;

        // preferenza: Qwen2.5-VL-7B (grounding pixel migliore), poi qwen3-vl-4b, poi gemma-3
        const preferred = [
            alive.models.includes(LLM_MODEL) ? LLM_MODEL : null,
            alive.models.find(id => /qwen2.5-vl/i.test(id)),
            alive.models.find(id => /qwen3-vl/i.test(id)),
            alive.models.find(id => /gemma-3/i.test(id))
        ].filter(Boolean);
        const tried = [];

        let lastErr = null;
        for (const model of preferred) {
            if (tried.includes(model)) continue;
            tried.push(model);

            const jsonTpl = '{"joints":[' +
                VISION_JOINT_LABELS.map(l => '{"label":"' + l + '","point":[0,0]}').join(',') +
                ']}';
            const userText =
                'Image size: 640x480 pixels. This photo shows a white 6-DOF robotic arm on a table.\n' +
                'The arm has 3 joints: BASE (bottom fixed pedestal), G1 (end of arm segment 1), G2 (end of arm segment 2).\n' +
                'Mark the exact pixel location of each joint with a point.\n' +
                'Return EXACTLY one JSON object and nothing else (no markdown fences, no commentary):\n' +
                jsonTpl + '\n' +
                'Rules:\n' +
                '- exactly ' + VISION_JOINTS_N + ' elements in that order: ' + VISION_JOINT_LABELS.join(', ') + '\n' +
                '- point = [x,y] integers; x 0..639 (left=0, right=639), y 0..479 (top=0, bottom=479)\n' +
                '- numbers only, never "-" or null\n' +
                '- BASE = bottom fixed pedestal of the arm; G1 = where arm segment 1 ends; G2 = where arm segment 2 ends (wrist side)\n' +
                '- fill each point with the ACTUAL pixel position of that joint in THIS image (the zeros above are placeholders, do not copy them)\n' +
                (attemptN > 1
                    ? '- ATTEMPT ' + attemptN + ': previous answer was WRONG' +
                      (feedback ? ' (' + String(feedback).slice(0, 300) + ')' : '') +
                      '. Re-inspect carefully.\n'
                    : '');

            const body = {
                model,
                messages: [
                    {
                        role: 'system',
                        content: 'You output ONLY minified JSON. No markdown. No explanation. Coordinates are pixel values on a 640x480 image.'
                    },
                    {
                        role: 'user',
                        content: [
                            { type: 'text', text: userText },
                            { type: 'image_url', image_url: { url: 'data:image/jpeg;base64,' + imagePart } }
                        ]
                    }
                ],
                max_tokens: 700,
                temperature: 0.05
            };

            let r;
            try {
                r = await httpJson(
                    'POST',
                    `http://${alive.host}:${alive.port}/v1/chat/completions`,
                    body,
                    LLM_TIMEOUT_MS
                );
            } catch (e) {
                lastErr = e.message;
                continue;
            }
            if (r.status !== 200 || !r.json) {
                lastErr = 'HTTP ' + r.status + ' ' + (r.text || '').slice(0, 200);
                continue;
            }
            const content = r.json.choices &&
                r.json.choices[0] &&
                r.json.choices[0].message &&
                r.json.choices[0].message.content;
            const joints = parseJointsFromText(content);
            if (joints) {
                pushLog(`🤖 LLM vision: ${VISION_JOINTS_N} giunti (tentativo ${attemptN}) via ${model}`);
                return res.json({
                    success: true,
                    joints,
                    attempt: attemptN,
                    model,
                    server: alive.name
                });
            }
            lastErr = 'parse fallito: ' + String(content || '').slice(0, 300);
        }

        return res.status(422).json({
            error: 'Risposta LLM non valida (mancano ' + VISION_JOINTS_N + ' giunti)',
            tried,
            detail: lastErr || ''
        });
    } catch (e) {
        res.status(500).json({ error: e.message });
    }
});

// Salva DOPPIA foto: pulita (senza giunti) + annotata (giunti + linee 3D)
// body: { imageClean, imageAnnotated, joints, q?, depthB64?, note? }
app.post('/api/vision/capture-pair', (req, res) => {
    try {
        const { imageClean, imageAnnotated, joints, q, depthB64, note } = req.body || {};
        const toBuf = (durl) => {
            if (!durl || typeof durl !== 'string' || durl.indexOf('base64') === -1) return null;
            const b64 = durl.slice(durl.indexOf(',') + 1);
            const buf = Buffer.from(b64, 'base64');
            return buf.length ? buf : null;
        };
        const cleanBuf = toBuf(imageClean);
        const annBuf = toBuf(imageAnnotated);
        if (!cleanBuf && !annBuf) {
            return res.status(400).json({ error: 'almeno una immagine dataURL base64 richiesta' });
        }

        const now = new Date();
        const stamp = now.toISOString().replace(/[:.]/g, '-').slice(0, 19);
        const id = 'cap_' + stamp + '_' + Date.now().toString(36);

        const imgCleanName = id + '.jpg';          // SENZA giunti (input ML)
        const imgAnnName = id + '_annotated.jpg';  // CON giunti + linee (label render)
        if (cleanBuf) fs.writeFileSync(path.join(VISION_DS, imgCleanName), cleanBuf);
        if (annBuf) fs.writeFileSync(path.join(VISION_DS, imgAnnName), annBuf);

        let depthPath = null;
        if (depthB64 && typeof depthB64 === 'string') {
            const dbuf = Buffer.from(depthB64, 'base64');
            if (dbuf.length) {
                depthPath = id + '.depth.bin';
                fs.writeFileSync(path.join(VISION_DS, depthPath), dbuf);
            }
        }

        const jointsArr = Array.isArray(joints) ? joints.map((j, i) => ({
            label: j.label || VISION_JOINT_LABELS[i],
            x: Number(j.x) || 0,
            y: Number(j.y) || 0,
            z: (j.z === undefined || j.z === null) ? null : Number(j.z)
        })) : [];

        const meta = {
            id,
            image: cleanBuf ? imgCleanName : imgAnnName,
            imageAnnotated: annBuf ? imgAnnName : null,
            depth: depthPath,
            created: now.toISOString(),
            q: Array.isArray(q) ? q.map(v => Number(v)) : null,
            note: note || 'llm-approve-pair',
            joints: jointsArr,
            approved: jointsArr.length === VISION_JOINTS_N,
            source: 'llm-vision-pair'
        };
        fs.writeFileSync(path.join(VISION_DS, id + '.json'), JSON.stringify(meta, null, 2));
        pushLog(`📸 Dataset ML doppia foto: ${meta.image} + ${meta.imageAnnotated || '—'} (${jointsArr.length} giunti)`);
        mlMaybeAutoTrain();   // auto-retrain se ci sono abbastanza campioni nuovi
        res.json({
            success: true,
            id,
            image: meta.image,
            imageAnnotated: meta.imageAnnotated,
            joints: jointsArr.length,
            count: visionListCaptures().length
        });
    } catch (e) {
        res.status(500).json({ error: e.message });
    }
});

app.get('/api/vision/image-annotated/:id', (req, res) => {
    const id = String(req.params.id).replace(/[^a-zA-Z0-9_\-]/g, '');
    const metaPath = path.join(VISION_DS, id + '.json');
    if (!fs.existsSync(metaPath)) return res.status(404).json({ error: 'non trovato' });
    try {
        const meta = JSON.parse(fs.readFileSync(metaPath, 'utf8'));
        const name = meta.imageAnnotated || (id + '_annotated.jpg');
        const img = path.join(VISION_DS, name);
        if (!fs.existsSync(img)) return res.status(404).json({ error: 'immagine annotata mancante' });
        res.sendFile(img);
    } catch (e) { res.status(500).json({ error: e.message }); }
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
