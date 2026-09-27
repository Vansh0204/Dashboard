# PneumoTrack SCADA 4.0
## Real-Time Pneumatic System Monitoring and Fault Detection Dashboard

**Department:** Mechatronics Engineering, School of Automation, Banasthali Vidyapith  
**Project Group:** Group 44 (Khushi Agarwal, Astha Singh, Mishti Khandelwal, Akansha Pandey, Himani Sahu)  
**Supervisor:** Dr. Vineet Pandey  

---

### 🌟 Project Architecture & Modifications
1. **Metal vs. Non-Metal Segregation Only:**
   - Color sorting removed from the original concept.
   - Segregation is performed via an **NPN-NO Inductive Proximity Sensor (8mm sensing distance, 6–36V DC)**.
   - Metallic specimens are detected and deflected by a **Janatics Double-Acting Pneumatic Cylinder (20mm Bore × 50mm Stroke)** into **Bin 1**.
   - Non-metallic specimens continue undisturbed along the conveyor to **Bin 2**.
2. **Industrial PLC Control (Replacing ESP32):**
   - The original 3.3V ESP32 was replaced with an **Industrial PLC (24V DC I/O)** to directly interface with 12V/24V solenoid valves, 6–36V inductive sensors, and industrial pressure transmitters without level-shifting bottlenecks.

---

### 📊 Dashboard Features
- **Live SCADA Telemetry:**
  - Real-time line pressure monitoring (bar / PSI).
  - Dynamic decay rate calculation ($\Delta P / \Delta t$) for **Pneumatic Leakage Detection**.
  - Cylinder stroke speed & cycle time counter.
  - Specimen sorting counts & efficiency metrics (Metals vs Non-Metals).
- **Interactive 2D Digital Twin:**
  - 60 FPS animated schematic view showing the conveyor belt, rotating rollers, moving metallic & non-metallic workpieces, inductive sensor magnetic flux animation, 5/2 single solenoid valve spool flow, pneumatic cylinder extension, and collection bins.
- **Fault Detection & Diagnosis (FDD) Engine:**
  - Highlights 5 faults specified in the project synopsis:
    1. `ERR-01`: Air Line Leakage ($\Delta P / \Delta t > 0.05 \text{ bar/s}$)
    2. `ERR-02`: Low Supply Pressure ($< 4.5 \text{ bar}$)
    3. `ERR-03`: Cylinder Non-Extension / Mechanical Jam ($t_{ext} > 250 \text{ ms}$)
    4. `ERR-04`: Sorting Failure / Bin Mismatch
    5. `ERR-05`: Sensor Abnormality / Stuck Signal
- **PLC Register Map (Modbus / 24V I/O):**
  - Displays live mapped memory tags:
    - `%IW100`: Analog Input (Pneumatic Pressure Sensor 0–10 bar)
    - `%IX0.0`: Digital Input (Inductive Proximity Sensor)
    - `%IX0.1`: Digital Input (Infeed IR Sensor)
    - `%IX0.2`: Digital Input (Bin 1 IR Sensor)
    - `%QX0.0`: Digital Output (5/2 Solenoid Valve Coil)
    - `%QX0.1`: Digital Output (Conveyor Motor 12V DC)
    - `%QX0.2`: Digital Output (Alarm Buzzer / Beacon)
- **Interactive Test-Bench / Viva Demonstrator:**
  - One-click fault injection buttons to demonstrate real-time alerts and root-cause diagnostics to examiners.
  - Manual workpiece injection (Metal / Non-Metal).
  - Web Audio synthesis: pneumatic solenoid hiss puff and industrial buzzer audio.
  - Export Shift Diagnostic Report (.CSV).

---

### 🚀 Running the Dashboard Locally

```bash
cd /Users/vanshagarwal/dashboard
npm install
npm run dev
```

Open [http://localhost:5173/](http://localhost:5173/) in your browser.

### 🔌 Connecting a Physical PLC

The dashboard runs on its internal simulation by default. To drive it from real hardware:

```bash
npm run bridge
```

This starts the Modbus TCP -> WebSocket bridge (`hardware_bridge.js`). Set your PLC's
IP in `PLC_CONFIG` at the top of that file first. Then in the dashboard set
**Feed Mode** to `websocket`.

Once live telemetry arrives, the internal simulation stands down: line pressure,
the measured decay rate (dP/dt), all sensor inputs, and the solenoid / conveyor
outputs come straight from the PLC. If the bridge goes quiet for 3 seconds the
dashboard automatically falls back to simulation.

**Why calibration is needed:** a PLC reports sensor bits, not workpiece positions.
The belt animation therefore *derives* workpieces from the infeed IR sensor's rising
edge and advances them at `state.conveyorSpeed` (canvas px/sec). If that value does
not match the real rig, the animated piece reaches the pusher at the wrong moment and
the twin reports phantom sort mismatches.

### 📏 Calibrating the Conveyor Speed

All belt geometry lives in one place — the `BELT_LAYOUT` constant at the top of
`src/main.js`, shared by both the physics and the renderer.

**Automatic (recommended).** With the bridge running and Feed Mode set to
`websocket`, the **Conveyor Speed Calibration** strip in the PLC panel times how
long each workpiece really takes to travel from the infeed IR to the inductive
sensor. Run at least 3 **metal** pieces through (only metal trips the inductive
sensor), then press **Apply**. It takes a median over up to 15 transits, so one
slipped or hand-fed piece will not skew the result.

To keep the value, copy the applied number into `conveyorSpeed` in `src/main.js`.

**Manual.** Measure the transit time `T` yourself and compute:

```
conveyorSpeed = 295 / T        // 295 px = infeed -> inductive in BELT_LAYOUT
```

**If your rig's proportions differ from the canvas.** A single speed can only align
both belt segments when the canvas layout is proportional to the machine. If it is
not, derive every anchor from one scale factor instead:

```
scale         = 470 / D_real(infeed -> pusher)        // px per mm
inductive.x   = 110 + scale * D_real(infeed -> inductive)
conveyorSpeed = scale * v_real
```

Then edit those values in `BELT_LAYOUT` — the renderer and the physics both follow.

**Sanity check:** feed metal pieces and confirm the animated block sits under the
pusher at the same moment the valve badge lights. Too far past = speed too high;
not yet arrived = too low.

> At the bridge's 10 Hz poll rate, timing resolution is 100 ms (~3% over a 3 s
> transit). If your belt is fast enough that transit drops below ~1 s, raise the
> poll rate first or the calibration is mostly noise.
