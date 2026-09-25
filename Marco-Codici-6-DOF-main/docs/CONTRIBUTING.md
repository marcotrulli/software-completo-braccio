# 🤝 Contributing Guidelines

## 📋 Come Contribuire

Benvenuto nel progetto Robot 6 DOF Control Center! Questo documento ti guiderà nel contribuire al progetto.

## 🚀 Setup di Sviluppo

### Prerequisiti
- Node.js 16+ (per backend Raspberry Pi)
- Visual Studio o .NET SDK (per backend PC C#)
- Browser moderno con supporto WebGL
- Git

### Setup Locale

1. **Clona il repository**:
   ```bash
   git clone https://github.com/TUO_USERNAME/robot6dof.git
   cd robot6dof
   ```

2. **Installa dipendenze Node.js**:
   ```bash
   npm install
   ```

3. **Testa il server**:
   ```bash
   # Su PC (Windows)
   scripts\TEST_PC_SERVER.bat
   
   # Su qualsiasi piattaforma con Node.js
   scripts\TEST_NODE_SERVER.bat
   ```

## 📁 Struttura del Progetto

```
robot6dof/
├── index.html              # App web principale (UI)
├── server.js               # Backend Node.js (Raspberry Pi)
├── Program.cs              # Backend C# (PC Windows)
├── config/                 # File JSON configurazione
├── models/                 # File STL 3D
├── firmware/               # Firmware ESP32
├── scripts/                # Script di setup e test
├── docs/                   # Documentazione
└── libs/                   # Librerie JavaScript
```

## 🔧 Regole di Sviluppo

### 1. Limiti Fisici Giunti
**Mai violare i limiti angolari:**
- Giunti dispari (J1, J3, J5): 270° span `[-135°, +135°]`
- Giunti pari (J2, J4, J6): 180° span
  - J2: `[-90°, +90°]`
  - J4: `[-135°, +45°]`
  - J6: `[-90°, +90°]`

### 2. Sincronizzazione File
Le modifiche devono essere sincronizzate tra:
- `index.html` (versione principale)
- `ik_simulator_v30.html` (versione corrente)
- `Robot6DOF_ESP32_Firmware.ino` (firmware ESP32)

### 3. Calcolo Vettoriale
**Non semplificare il calcolo delle forze:**
- Usa sempre prodotto vettoriale: `r × F`
- Proiezione sugli assi di rotazione reali
- Non usare somme scalari approssimate

### 4. Percorsi File
Quando sposti file, aggiorna sempre i percorsi in:
- `server.js` (backend Node.js)
- `Program.cs` (backend C#)
- Script in `scripts/`

## 🧪 Testing

### Test Backend
```bash
# Test server Node.js
npm start
npm run test-esp32

# Test server C# PC
scripts\TEST_PC_SERVER.bat
scripts\AVVIA_SIMULATORE.bat
```

### Test UI
1. Apri `http://localhost:8765`
2. Verifica tutti i pannelli funzionano
3. Testa calcoli IK/FK
4. Verifica fisica e FEA
5. Testa comunicazione ESP32 (su Raspberry Pi)

## 📝 Commit Guidelines

### Formato Commit
```
<Tipo>: <Descrizione breve>

<Descrizione dettagliata se necessario>
```

### Tipi di Commit
- `feat`: Nuova funzionalità
- `fix`: Bug fix
- `docs`: Documentazione
- `style`: Formattazione codice
- `refactor`: Refactoring
- `test`: Test
- `chore`: Manutenzione

### Esempi
```
feat: aggiungere supporto nuova modalità IK
fix: correggere calcolo torque giunto J4
docs: aggiornare README con istruzioni Raspberry Pi
refactor: riorganizzare struttura cartelle config/
```

## 🔄 Workflow Pull Request

1. **Fork il repository**
2. **Crea branch feature**: `git checkout -b feature/tua-feature`
3. **Commit changes**: `git commit -m "feat: descrizione"`
4. **Push to branch**: `git push origin feature/tua-feature`
5. **Apri Pull Request**

### PR Checklist
- [ ] Codice segue le regole del progetto
- [ ] Test eseguiti con successo
- [ ] Documentazione aggiornata
- [ ] Commits seguono le guidelines
- [ ] Nessun file sensibile incluso

## 🐛 Segnalazione Bug

Usa il template GitHub Issues:

```markdown
**Descrizione**
Breve descrizione del bug

**Passi per riprodurre**
1. Vai a...
2. Clicca su...
3. Vedi errore...

**Comportamento atteso**
Dovrebbe succedere...

**Screenshot**
Se applicabile

**Ambiente**
- OS: [es. Windows 11, Raspberry Pi OS]
- Browser: [es. Chrome, Firefox]
- Server: [C# / Node.js]
```

## 💡 Suggerimenti Funzionalità

Per nuove funzionalità, apri una Issue con tag `enhancement` e descrivi:
- Caso d'uso
- Funzionalità proposta
- Possibile implementazione
- Impatto su codice esistente

## 📄 Licenza

Contribuendo accetti che il tuo codice sarà rilasciato sotto la licenza MIT del progetto.

## 🆘 Supporto

Per domande:
- Controlla `docs/README.md`
- Cerca nelle Issues esistenti
- Apri nuova Issue con tag `question`

Grazie per il tuo contributo! 🙏
