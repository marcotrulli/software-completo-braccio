# ROBOT 6 DOF - DIGITAL TWIN & EMBEDDED CONTROL ECOSYSTEM
> **SYSTEM PROMPT & SPECIFICHE TECNICHE COMPLETE DI PROGETTO**  
> *Marco Codici 6 DOF · Workspace Studio*  
> *Versione di Riferimento: v30 (Fisica Avanzata, Stress Analitico, FEA Materiali, C-Space Trajectory Pathfinder, ESP32 Firmware)*

---

## 1. 📌 PANORAMICA DEL PROGETTO
Questo ecosistema software e hardware costituisce una piattaforma completa per la progettazione, simulazione fotorealistica 3D, calcolo strutturale/fisico in tempo reale (coppie gravitazionali $\tau$ e stress Von Mises agli elementi finiti FEA), generazione di traiettorie fluide sicure anti-sovraccarico e controllo embedded su microcontrollore **ESP32** di un **braccio robotico articolato a 6 Gradi di Libertà (6 DOF)**.

```
       [ WEB CLIENT / UI STUDIO 3D ]                    [ HARDWARE REALE ]
   ┌────────────────────────────────────┐             ┌─────────────────────┐
   │  • Three.js WebGL Engine (v30)     │             │  • Microcontrollore │
   │  • Risolutore IK 6 DOF Ibrido      │             │    ESP32            │
   │  • Calcolo Torque Vettoriale & FEA │             │  • Driver PCA9685 / │
   │  • Pathfinder Ricorsivo C-Space    │             │    GPIO Diretti     │
   │  • Editor Animazioni & Pose JSON   │             │  • 6 Servomotori    │
   └─────────────────┬──────────────────┘             │    (180° / 270°)    │
                     │                                └──────────▲──────────┘
                     │ Esportazione C++ (.ino)                   │
                     └───────────────────────────────────────────┘
```

---

## 2. 🗂️ ARCHITETTURA FILE E STRUTTURA DEL WORKSPACE

| Nome File | Descrizione e Ruolo nel Progetto |
| :--- | :--- |
| **`index.html`** / **`ik_simulator_v30.html`** | Applicazione principale Digital Twin: grafica 3D Three.js, controlli CAD, risolutore IK/FK, motore fisico, FEA, scia 3D, generatore nuvola di raggiungibilità e sequencer animazioni. |
| **`Robot6DOF_ESP32_Firmware.ino`** | Firmware C++ Arduino completo e standalone per ESP32. Integra cinematica inversa ibrida, S-curve cubic spline, protezione sovraccarico e CLI seriale a 115200 baud. |
| **`DigitalTwin6DOF.exe`** | Web server HTTP locale autonomo in Go/C per servire l'applicazione su `http://localhost:8765/` senza dipendenze Node.js. |
| **`AVVIA_SIMULATORE.bat`** | Script di avvio rapido in un click (avvia il web server locale e apre il browser predefinito). |
| **`robot6dof_project.json`** | File di configurazione salvato su disco (geometrie, limiti, motori, materiali, pesi STL, impostazioni scia). |
| **`robot6dof_animations.json`** | Archivio delle routine e sequenze di animazione (tempi, curve di interpolazione, pose collegate). |
| **`robot6dof_saved_poses.json`** | Libreria pose statiche denominate memorizzate dal designer. |
| **`libs/`** | Librerie offline incorporate: `three.min.js`, `STLLoader.js`, `TransformControls.js`. |
| **`*.stl`** (8 Mesh 3D) | File CAD originali dei link del braccio: `base_finita.stl`, `coperchio_base_rotante.stl`, `collegamento_giunti_braccio_1.stl`, `braccio_1.stl`, `collegamento_giunti_braccio_2.stl`, `braccio_2.stl`, `collegamento_giunti_braccio_3.stl`, `braccio_fine.stl`. |
| **`DigitalTwin6DOF_Portable.zip`** | Pacchetto zip autosufficiente e portatile contenente tutti gli asset, firmware ed eseguibili. |

---

## 3. ⚙️ REGOLA FERREA DEI LIMITI ANGOLARI DEI GIUNTI (180° / 270°)

I 6 servomotori fisici montati sul braccio robotico appartengono a due classi distinte di escursione:
- **Giunti DISPARI ($J_1, J_3, J_5$)** $\to$ **Servi a 270° di libertà**:
  - $J_1$ (Base Yaw): $[-135.0^\circ, +135.0^\circ]$ $\to$ Span totale: $270^\circ$
  - $J_3$ (Braccio Roll): $[-135.0^\circ, +135.0^\circ]$ $\to$ Span totale: $270^\circ$
  - $J_5$ (Polso Roll): $[-135.0^\circ, +135.0^\circ]$ $\to$ Span totale: $270^\circ$
- **Giunti PARI ($J_2, J_4, J_6$)** $\to$ **Servi a 180° di libertà**:
  - $J_2$ (Spalla Pitch): $[-90.0^\circ, +90.0^\circ]$ $\to$ Span totale: $180^\circ$
  - $J_4$ (Gomito Pitch): $[-135.0^\circ, +45.0^\circ]$ $\to$ Span totale: $180^\circ$ (permette la flessione profonda ad arco fino a $-135^\circ$)
  - $J_6$ (Pinza Pitch): $[-90.0^\circ, +90.0^\circ]$ $\to$ Span totale: $180^\circ$

```javascript
const limits = [
  [-135.0, 135.0], // J1 Dispari (270°)
  [ -90.0,  90.0], // J2 Pari (180°)
  [-135.0, 135.0], // J3 Dispari (270°)
  [-135.0,  45.0], // J4 Pari (180°)
  [-135.0, 135.0], // J5 Dispari (270°)
  [ -90.0,  90.0]  // J6 Pari (180°)
];
```

---

## 4. 📐 GEOMETRIA DEL BRACCIO E PARAMETRI STRUTTURALI (mm)

```
       (Z)
        ▲
        │          [J3 Roll]     [J4 Pitch Gomito]
        │              ▲                 ●━━━━━━━┓ L2 = 220mm
        │              │  L1 = 220mm             ┃
        │              ●                         ┃
        │      [J2 Pitch Spalla]                 ● [J5 Roll]
        │              │                         │
        │              │ d12 = 50mm              ● [J6 Pitch Pinza]
        │              ▼                         │
     ━━━●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━▼━━━━━━━ (X/Y)
     [J1 Yaw Base]                          [Punta Pinza Endpoint]
                                             Lg = 150mm
```

- $d_{12} = 50\,\text{mm}$ (Offset verticale da Base a Asse Spalla $J_2$)
- $L_1 = 220\,\text{mm}$ (Lunghezza primo braccio superiore)
- $d_{34} = 50\,\text{mm}$ (Distanziatore tra torsione $J_3$ e gomito $J_4$)
- $L_2 = 220\,\text{mm}$ (Lunghezza avambraccio)
- $d_{56} = 50\,\text{mm}$ (Distanziatore tra polso $J_5$ e pitch pinza $J_6$)
- $L_g = 150\,\text{mm}$ (Lunghezza corpo pinza ed end-effector)
- $R_{\text{max}} = L_1 + L_2 + L_g = 590\,\text{mm}$ (Raggio di sbraccio massimo teorico)

---

## 5. 🔬 MODELLO FISICO, COPPIE GRAVITAZIONALI E ANALISI FEA DEI MATERIALI

### A. Calcolo Vettoriale Esatto delle Coppie ($\text{kg}\cdot\text{cm}$)
Per ciascun giunto $j \in [1 \dots 6]$, la coppia gravitazionale netta è calcolata tramite sommatoria dei momenti vettoriali di tutte le masse a valle:
$$\vec{\tau}_j = \sum_{k \ge j} \vec{r}_{k/j} \times (m_k \cdot \vec{g})$$
dove $\vec{r}_{k/j} = \text{CoM}_k - \text{Pos}_j$, e $m_k$ include il peso del link stampato 3D, il peso del motore alloggiato e il carico (payload) afferrato dalla pinza.

La coppia scalare attiva agente sull'albero del servomotore è la proiezione sul suo asse di rotazione reale $\hat{u}_j$:
$$\tau_{\text{attiva}, j} = \left| \vec{\tau}_j \cdot \hat{u}_j \right|$$
$$\text{Stress Motor}\% = \left( \frac{\tau_{\text{attiva}, j}}{\tau_{\text{nominale}, j}} \right) \cdot 100$$

### B. Analisi FEA Strutturale e Sollecitazioni di Von Mises ($\text{MPa}$)
Per ogni sezione del braccio soggetta a momento flettente $M_f$, lo stress massimo e il fattore di sicurezza sono:
$$\sigma_{\text{bending}} = \frac{M_f \cdot y_{\text{max}}}{I_z} = \frac{32 \cdot M_f}{\pi \cdot d^3}$$
$$\sigma_{\text{Von Mises}} = \sqrt{\sigma_{\text{bending}}^2 + 3 \cdot \tau_{\text{torsion}}^2}$$
$$\text{Fattore di Sicurezza } S_f = \frac{\sigma_{\text{snervamento, materiale}}}{\sigma_{\text{Von Mises}}}$$

Libreria Materiali Supportati:
- **PLA**: $\sigma_y = 50.0\,\text{MPa}$, Modulo $E = 3.5\,\text{GPa}$, Densità $\rho = 1.24\,\text{g/cm}^3$
- **PETG**: $\sigma_y = 45.0\,\text{MPa}$, Modulo $E = 2.1\,\text{GPa}$, Densità $\rho = 1.27\,\text{g/cm}^3$
- **ABS**: $\sigma_y = 40.0\,\text{MPa}$, Modulo $E = 2.3\,\text{GPa}$, Densità $\rho = 1.04\,\text{g/cm}^3$
- **PC (Policarbonato)**: $\sigma_y = 65.0\,\text{MPa}$, Modulo $E = 2.4\,\text{GPa}$, Densità $\rho = 1.20\,\text{g/cm}^3$
- **PA-CF (Nylon Carbonio)**: $\sigma_y = 85.0\,\text{MPa}$, Modulo $E = 6.0\,\text{GPa}$, Densità $\rho = 1.15\,\text{g/cm}^3$

---

## 6. 🧠 CINEMATICA INVERSA (IK) IBRIDA & POSTURE A MINIMO STRESS

Il risolutore combina la **Cinematica Analitica Planare Trigonometrica a 3 Link** con un ottimizzatore **Damped Least Squares (DLS)** vincolato su matrice Jacobiana $6 \times 6$:

```
                             [ Target XYZ & Orientamento Pinza ]
                                              │
                    ┌─────────────────────────┴─────────────────────────┐
                    ▼                                                   ▼
        [ Risolutore Analitico ]                             [ DLS Jacobian $6 \times 6$ ]
      Triangolazione Polare 3-Link                   $\Delta q = (J^T J + \lambda I)^{-1} J^T e$
                    │                                                   │
                    └─────────────────────────┬─────────────────────────┘
                                              ▼
                             [ Cost-Function & Min-J2 Torque ]
                       Cost += $\text{Stress}_{J_2}^{1.8} \times 20 + \text{OverloadPenalties}$
                                              ▼
                             [ Soluzione Cinematica Ottimale ]
```

### Priorità Cinematica: Postura a Gru (Crane Arch) & $J_2 \approx 0^\circ$
Per evitare che il motore $J_2$ (spalla) collassi sotto carichi pesanti, l'IK premia le configurazioni dove:
1. $J_2$ rimane vicino alla verticale ($-10^\circ \le J_2 \le +20^\circ$).
2. Il gomito $J_4$ sale ad arco al di sopra del polso ($Z_{\text{gomito}} > Z_{\text{polso}}$).
3. La pinza $J_6$ si livella in orizzontale in modo automatico.

---

## 7. 🛡️ PIANIFICATORE DI TRAIETTORIE C-SPACE (Recursive Bisection Pathfinder)

Per evitare che durante il transito tra due pose $A$ e $B$ i motori vadano in stallo ($> 99\%$ stress):
1. L'algoritmo campiona la traiettoria e calcola la fisica di ogni punto intermedio.
2. Se un punto intermedio supera la soglia di sicurezza ($65\%$), il pianificatore genera un **Waypoint Correttivo di Transito Sicuro**:
   - $J_2 \to 0^\circ \dots 15^\circ$ (Raddrizzamento spalla).
   - $J_4 \le -85^\circ$ (Chiusura gomito a compasso per azzerare il braccio di leva $r \times F$).
3. Applica una **Bisezione Ricorsiva** verificando che tutte le sotto-tratte siano prive di picchi di carico.
4. Esegue il movimento tramite interpolazione spline cubica continua multi-segmento.

---

## 8. ⚡ ESP32 FIRMWARE: PROTOCOLLO SERIALE A 115200 BAUD

Il file `Robot6DOF_ESP32_Firmware.ino` accetta comandi ASCII tramite monitor seriale:

- `IK <X> <Y> <Z> [0|1]` $\to$ Esegue la cinematica inversa e muove la pinza alle coordinate millimetriche ($1$ = orizzontale, $0$ = libera).
  *Esempio*: `IK 150.0 200.0 120.0 1`
- `MOVE <J1> <J2> <J3> <J4> <J5> <J6>` $\to$ Movimento diretto dei 6 giunti in gradi con curva $S\text{-Curve}$.
  *Esempio*: `MOVE 0 15 0 -85 0 70`
- `REST` $\to$ Riporta il robot nella posa di riposo ottimizzata `[0, 47, 0, -69, 7, -68]`.
- `STATUS` o `POS` $\to$ Stampa coordinate $XYZ$, angoli attuali e stress motori.
- `ATTACH` / `DETACH` $\to$ Abilita o disabilita i segnali PWM dei servomotori.
- `HELP` $\to$ Mostra il menu dei comandi.

---

## 9. 🤖 LINEE GUIDA PER LO SVILUPPO E FUTURE INTEGRAZIONI AI

Quando un'altra istanza AI o uno sviluppatore apre questo progetto, deve attenersi alle seguenti regole:

1. **Non violare mai i limiti fisici dei giunti**: I dispari hanno $270^\circ$ di span ($[-135, +135]$), i pari hanno $180^\circ$ di span ($J_2 \in [-90, +90]$, $J_4 \in [-135, +45]$, $J_6 \in [-90, +90]$).
2. **Mantenere la sincronizzazione dei file**: Qualsiasi modifica logica a `ik_simulator_v30.html` deve essere sincronizzata su `index.html`, `ik_simulator_v29.html` e riflessa nel firmware `Robot6DOF_ESP32_Firmware.ino` e nel file `DigitalTwin6DOF_Portable.zip`.
3. **Mantenere il calcolo vettoriale reale delle forze**: Non semplificare il calcolo dei momenti torcenti con somme scalari; usare sempre il prodotto vettoriale $\vec{r} \times \vec{F}$ proiettato sugli assi di rotazione Three.js / Euler.
4. **Preservare la persistenza locale**: L'applicazione salva configurazioni su IndexedDB e su file JSON tramite File System Access API.
