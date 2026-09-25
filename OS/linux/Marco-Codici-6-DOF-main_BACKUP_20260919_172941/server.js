const express = require('express');
const { SerialPort } = require('serialport');
const cors = require('cors');
const path = require('path');
const fs = require('fs');

const app = express();
const PORT = 8765;

// Middleware
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

// Configurazione porta seriale ESP32
let serialPort = null;
let esp32Connected = false;
const ESP32_BAUDRATE = 115200;

// Funzione per connettersi alla porta seriale
async function connectToESP32() {
    try {
        // Cerca automaticamente le porte seriali disponibili
        const ports = await SerialPort.list();
        console.log('Porte seriali disponibili:', ports.map(p => p.path));

        // Cerca porte tipiche ESP32 (ttyUSB0, ttyACM0, COMx su Windows)
        const esp32Port = ports.find(p => 
            p.path.includes('USB') || 
            p.path.includes('ACM') || 
            p.path.includes('COM')
        );

        if (!esp32Port) {
            console.log('Nessuna porta ESP32 trovata, modalità solo simulazione');
            return false;
        }

        serialPort = new SerialPort({
            path: esp32Port.path,
            baudRate: ESP32_BAUDRATE,
            autoOpen: false
        });

        // Wait for port to open properly using Promise
        await new Promise((resolve, reject) => {
            serialPort.open((err) => {
                if (err) {
                    console.error('Errore apertura porta seriale:', err.message);
                    esp32Connected = false;
                    reject(err);
                } else {
                    console.log(`✅ Connesso a ESP32 su ${esp32Port.path}`);
                    esp32Connected = true;
                    resolve();
                }
            });
        });

        serialPort.on('error', (err) => {
            console.error('Errore seriale:', err.message);
            esp32Connected = false;
        });

        serialPort.on('close', () => {
            console.log('Connessione seriale chiusa');
            esp32Connected = false;
        });

        return true;
    } catch (error) {
        console.error('Errore connessione ESP32:', error.message);
        return false;
    }
}

// API: Stato connessione ESP32
app.get('/api/esp32/status', (req, res) => {
    res.json({
        connected: esp32Connected,
        port: serialPort ? serialPort.path : null
    });
});

// API: Invia comando all'ESP32
app.post('/api/esp32/send', async (req, res) => {
    const { command } = req.body;

    if (!command) {
        return res.status(400).json({ error: 'Comando mancante' });
    }

    if (!esp32Connected || !serialPort) {
        return res.status(503).json({ 
            error: 'ESP32 non connesso',
            message: 'Collega ESP32 e riavvia il server'
        });
    }

    try {
        const cmdString = command + '\n';
        serialPort.write(cmdString, (err) => {
            if (err) {
                return res.status(500).json({ error: err.message });
            }
            console.log(`📤 Inviato a ESP32: ${command}`);
            res.json({ success: true, command });
        });
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});

// API: Lista porte seriali disponibili
app.get('/api/serial/ports', async (req, res) => {
    try {
        const ports = await SerialPort.list();
        res.json({ ports: ports.map(p => p.path) });
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});

// API: Riconnessione ESP32
app.post('/api/esp32/reconnect', async (req, res) => {
    if (serialPort && serialPort.isOpen) {
        serialPort.close();
    }
    const success = await connectToESP32();
    res.json({ success, connected: esp32Connected });
});

// API: Logs seriale ESP32
app.get('/api/esp32/logs', (req, res) => {
    // Return recent serial communication logs
    // In a real implementation, this would read from the serial port buffer
    // For now, return status-based information
    if (esp32Connected && serialPort && serialPort.isOpen) {
        res.json({ 
            logs: [`✅ ESP32 connesso su ${serialPort.path}`],
            connected: true 
        });
    } else {
        res.json({ 
            logs: ['⚠️ ESP32 non connesso - modalità simulazione'],
            connected: false 
        });
    }
});

// API: Salva configurazioni (stesso del server C#)
app.post('/api/save_stl_config', (req, res) => {
    try {
        saveConfigFile('robot6dof_stl_config.json', req.body);
        res.json({ status: 'ok' });
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});

app.post('/api/save_poses_config', (req, res) => {
    try {
        saveConfigFile('robot6dof_saved_poses.json', req.body);
        res.json({ status: 'ok' });
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});

app.post('/api/save_anim_config', (req, res) => {
    try {
        saveConfigFile('robot6dof_animations.json', req.body);
        res.json({ status: 'ok' });
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});

app.get('/api/list_stls', (req, res) => {
    try {
        const files = fs.readdirSync(path.join(__dirname, 'models'))
            .filter(f => f.endsWith('.stl'));
        res.json(files);
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});

// Avvio server
async function startServer() {
    console.log('🚀 Avvio Robot 6 DOF Control Center...');
    
    // Tentativo connessione ESP32
    await connectToESP32();

    app.listen(PORT, '0.0.0.0', () => {
        console.log(`✅ Server attivo su http://localhost:${PORT}`);
        console.log(`🌐 Accessibile da rete: http://<IP-RASPBERRY>:${PORT}`);
        console.log(`📡 ESP32 Status: ${esp32Connected ? 'CONNESSO' : 'NON CONNESSO (modo simulazione)'}`);
    });
}

startServer();
