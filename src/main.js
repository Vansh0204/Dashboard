/**
 * PNEUMOTRACK SCADA 4.0 - REAL-TIME PNEUMATIC MONITORING & FAULT DETECTION
 * B.Tech Mechatronics - Group 44 (Banasthali Vidyapith)
 * Supervisor: Dr. Vineet Pandey
 * 
 * Hardware Architecture:
 * - Industrial 24V PLC Logic (Replacing 3.3V ESP32 for 12V/24V Solenoid & Sensor compatibility)
 * - NPN-NO Inductive Proximity Sensor (8mm sensing distance) -> Metal Detection
 * - Analog Pressure Sensor (0-10 bar, 4-20mA / 0-10V)
 * - 5/2 Single Solenoid Valve (12V/24V coil, 0-8 bar)
 * - Janatics 20mm Bore x 50mm Stroke Double Acting Cylinder
 * - Conveyor Motor (12V Geared DC) & Dual IR Verification Sensors
 */

// ============================================================================
// AUDIO SYNTHESIS ENGINE (Web Audio API - No external assets required)
// ============================================================================
class AudioSynthesizer {
  constructor() {
    this.ctx = null;
    this.enabled = true;
  }

  init() {
    if (!this.ctx) {
      const AudioCtx = window.AudioContext || window.webkitAudioContext;
      if (AudioCtx) {
        this.ctx = new AudioCtx();
      }
    }
  }

  toggle() {
    this.enabled = !this.enabled;
    return this.enabled;
  }

  // Pneumatic release / air puff hiss
  playPneumaticHiss() {
    if (!this.enabled) return;
    this.init();
    if (!this.ctx) return;

    try {
      const bufferSize = this.ctx.sampleRate * 0.15;
      const buffer = this.ctx.createBuffer(1, bufferSize, this.ctx.sampleRate);
      const output = buffer.getChannelData(0);
      for (let i = 0; i < bufferSize; i++) {
        output[i] = (Math.random() * 2 - 1) * Math.exp(-i / (bufferSize * 0.3));
      }

      const whiteNoise = this.ctx.createBufferSource();
      whiteNoise.buffer = buffer;

      const filter = this.ctx.createBiquadFilter();
      filter.type = 'bandpass';
      filter.frequency.value = 1800;
      filter.Q.value = 1.2;

      const gain = this.ctx.createGain();
      gain.gain.setValueAtTime(0.25, this.ctx.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.001, this.ctx.currentTime + 0.15);

      whiteNoise.connect(filter);
      filter.connect(gain);
      gain.connect(this.ctx.destination);
      whiteNoise.start();
    } catch {
      // AudioContext policy fallback
    }
  }

  // Inductive metal detection chirp
  playMetalBeep() {
    if (!this.enabled) return;
    this.init();
    if (!this.ctx) return;

    try {
      const osc = this.ctx.createOscillator();
      const gain = this.ctx.createGain();
      osc.type = 'sine';
      osc.frequency.setValueAtTime(880, this.ctx.currentTime);
      osc.frequency.exponentialRampToValueAtTime(1320, this.ctx.currentTime + 0.08);

      gain.gain.setValueAtTime(0.15, this.ctx.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.001, this.ctx.currentTime + 0.08);

      osc.connect(gain);
      gain.connect(this.ctx.destination);
      osc.start();
      osc.stop(this.ctx.currentTime + 0.08);
    } catch {}
  }

  // Industrial Warning Alarm Buzzer
  playAlarmBuzzer() {
    if (!this.enabled) return;
    this.init();
    if (!this.ctx) return;

    try {
      const osc = this.ctx.createOscillator();
      const gain = this.ctx.createGain();
      osc.type = 'sawtooth';
      osc.frequency.setValueAtTime(440, this.ctx.currentTime);
      osc.frequency.setValueAtTime(330, this.ctx.currentTime + 0.1);

      gain.gain.setValueAtTime(0.2, this.ctx.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.01, this.ctx.currentTime + 0.22);

      osc.connect(gain);
      gain.connect(this.ctx.destination);
      osc.start();
      osc.stop(this.ctx.currentTime + 0.22);
    } catch {}
  }
}

// ============================================================================
// SYSTEM STATE & CONFIGURATION
// ============================================================================
// ============================================================================
// DIGITAL TWIN BELT GEOMETRY (canvas pixels)
// ============================================================================
// Single source of truth shared by the physics in PneumaticSimulationEngine and
// by the canvas renderer. To calibrate against a real rig, derive every x from
// measured mm distances using ONE px-per-mm scale so the twin stays
// proportional to the machine (see the calibration section in README.md).
const BELT_LAYOUT = {
  spawnX: 30,
  beltStartX: 20,
  beltEndX: 1040,
  beltY: 165,
  beltH: 50,

  infeedIR: { x: 110, halfWidth: 22 },
  inductive: { x: 405, halfWidth: 20 },   // 8mm NPN-NO sensing window

  pusher: {
    x: 580,               // cylinder / 5-2 valve centre line
    zoneStart: 540,       // blade contact window
    zoneEnd: 625,
    preTriggerStart: 520, // simulated mode fires the valve here
    preTriggerEnd: 610
  },

  bin1: { x: 540, y: 245, w: 100, h: 105 },
  bin2: { x: 1045, y: 230, w: 110, h: 115 },
  bin2DropX: 1040
};

// Distance a workpiece travels between the two sensors that the PLC actually
// reports. This is the baseline the conveyor-speed calibration measures against.
const INFEED_TO_INDUCTIVE_PX = BELT_LAYOUT.inductive.x - BELT_LAYOUT.infeedIR.x;

function inInductiveWindow(x) {
  return Math.abs(x - BELT_LAYOUT.inductive.x) <= BELT_LAYOUT.inductive.halfWidth;
}

const state = {
  // Operational Modes
  isRunning: true,
  isEmergencyStop: false,
  feedIntervalSec: 3.0,
  feedTimer: 0,
  feedMode: 'simulated', // 'simulated' | 'websocket' | 'modbus'

  // Live PLC link (fed by hardware_bridge.js over WebSocket)
  live: {
    connected: false,
    lastPacket: 0,
    lastInfeedIR: false,
    lastSolenoid: false,
    lastInductive: false
  },

  // Conveyor speed auto-calibration. Times how long a workpiece really takes to
  // travel from the infeed IR to the inductive sensor, which is the only
  // transit the PLC can actually observe, and converts it to canvas px/sec.
  calibration: {
    samples: [],          // transit times, seconds
    infeedEdgeAt: 0,      // performance.now() of the last infeed rising edge
    suggestedSpeed: null
  },

  // Faults deliberately injected from the test bench, as opposed to faults the
  // FDD engine detected on its own. Injected faults latch until the operator
  // clears them; detected faults are allowed to self-clear.
  injected: {
    airleak: false,
    lowpressure: false,
    cylinderext: false,
    sortingmismatch: false,
    sensorfault: false
  },

  // Pneumatic Physics
  nominalSupplyPressure: 6.2, // bar
  actualPressure: 6.2,       // bar (with micro-noise)
  pressureHistory: new Array(120).fill(6.2),
  pressureDecayRate: 0.012,  // bar/s (calculated ΔP/Δt)
  pressureThresholdMin: 4.5, // Min bar threshold for cylinder stroke
  airConsumptionNL: 14.2,    // Normal Liters / min

  // Cylinder & Solenoid Valve
  solenoidEnergized: false,
  cylinderPos: 0.0,          // 0.0 (retracted) -> 1.0 (extended)
  cylinderTarget: 0.0,
  cylinderStrokeTimeMs: 118,
  cylinderActuations: 0,
  lastStrokeStart: 0,

  // Conveyor & Specimen Tracker
  conveyorSpeed: 110,         // px/sec (canvas). On a live link this must be
                              // calibrated so a piece takes the same time to
                              // travel infeed -> pusher as it does on the rig,
                              // otherwise the animated piece drifts out of sync
                              // with the PLC's valve timing.
  conveyorRunning: true,
  specimens: [],             // array of { id, type: 'metal'|'nonmetal', x, y, state }
  nextSpecimenId: 1,

  // Sensor States (PLC I/O)
  sensors: {
    inductive: false,        // %IX0.0 (High when metal in 8mm range)
    irInfeed: false,         // %IX0.1 (Object enters conveyor)
    irBin1: false,           // %IX0.2 (Verified in Metal collection bin)
    irBin2: false            // %IX0.3 (Verified in Non-Metal bin)
  },

  // Sorting Counters
  counts: {
    metal: 0,
    nonmetal: 0,
    total: 0,
    missed: 0
  },

  // Active Fault Flags (PDF FDD Criteria)
  faults: {
    airleak: false,           // Decay > 0.05 bar/s
    lowpressure: false,       // Line < 4.5 bar
    cylinderext: false,       // Cylinder stuck / delayed stroke
    sortingmismatch: false,   // Object missed bin verification
    sensorfault: false        // Inductive sensor stuck
  },

  // Alarm Log
  alarmLog: [
    {
      time: new Date().toLocaleTimeString(),
      code: 'SYS-001',
      type: 'info',
      msg: 'PLC Controller online. Pneumatic line initialized at 6.2 bar nominal.'
    }
  ]
};

const audio = new AudioSynthesizer();

// True when the dashboard is being driven by real PLC telemetry instead of the
// internal simulation. Gates every block that would otherwise fabricate data.
function isLiveTelemetry() {
  return state.feedMode === 'websocket' && state.live.connected;
}

// ============================================================================
// FAULT DETECTION & DIAGNOSTICS (FDD) ENGINE
// ============================================================================
class FaultDiagnosticEngine {
  constructor() {
    this.lastPressure = state.actualPressure;
    this.lastCheckTime = performance.now();
    this.leakEvaluationInterval = 1000; // ms
    this.lastLeakCheck = performance.now();
    this.alarmBuzzerInterval = 0;
  }

  update(now, dt) {
    if (state.isEmergencyStop) return;

    // 1. Air Leakage Evaluation (Only evaluate when line is steady and idle, not during stroke)
    const isLineIdle = (now - state.lastStrokeStart) > 2000;
    if (now - this.lastLeakCheck >= this.leakEvaluationInterval) {
      const deltaT = (now - this.lastLeakCheck) / 1000;
      const deltaP = this.lastPressure - state.actualPressure;
      this.lastLeakCheck = now;
      this.lastPressure = state.actualPressure;

      if (isLiveTelemetry()) {
        // Genuine dP/dt from the PLC's analog pressure input over the real
        // sampling interval - no synthetic value on a live link.
        const measured = deltaT > 0 ? deltaP / deltaT : 0;
        state.pressureDecayRate = parseFloat(Math.max(0, measured).toFixed(3));
        if (isLineIdle && state.pressureDecayRate > 0.05) {
          this.raiseFault('airleak', 'ERR-01: AIR LEAKAGE', `Measured line decay ${state.pressureDecayRate} bar/s exceeds the 0.05 bar/s tolerance. Inspect tube fittings & valve seals.`);
        } else if (isLineIdle && !state.injected.airleak) {
          this.clearFault('airleak');
        }
      } else if (state.faults.airleak) {
        // True injected leak
        state.pressureDecayRate = parseFloat((0.18 + Math.random() * 0.03).toFixed(3));
        this.raiseFault('airleak', 'ERR-01: AIR LEAKAGE', 'Pneumatic line pressure decay rate exceeds tolerance (> 0.05 bar/s). Inspect tube fittings & valve seals.');
      } else if (isLineIdle) {
        // Normal steady state: healthy minimal loss (<0.015 bar/s)
        const idleDecay = Math.max(0.006, Math.min(0.018, 0.010 + (Math.random() - 0.5) * 0.004));
        state.pressureDecayRate = parseFloat(idleDecay.toFixed(3));
        this.clearFault('airleak');
      }
    }

    // 2. Low Supply Pressure Evaluation (< 4.5 bar)
    if (state.actualPressure < state.pressureThresholdMin) {
      this.raiseFault('lowpressure', 'ERR-02: LOW PRESSURE', `Supply pressure (${state.actualPressure.toFixed(2)} bar) below critical 4.5 bar threshold. Risk of actuator stalling.`);
    } else if (!state.injected.lowpressure) {
      this.clearFault('lowpressure');
    }

    // 3. Cylinder Non-Extension / Stroke Sticking Evaluation
    // Only meaningful while the valve is actively COMMANDING extension. During
    // the retract phase the solenoid is still energized but cylinderPos is
    // legitimately falling back below 0.8, which used to raise a false jam.
    if (state.solenoidEnergized && state.cylinderTarget === 1.0) {
      const elapsed = performance.now() - state.lastStrokeStart;
      if (elapsed > 250 && state.cylinderPos < 0.8) {
        this.raiseFault('cylinderext', 'ERR-03: CYLINDER STUCK', '5/2 Solenoid energized but pneumatic cylinder failed full 50mm stroke within 250ms timeout.');
      } else if (state.cylinderPos >= 0.98 && !state.injected.cylinderext) {
        // Full 50mm stroke achieved - a previously detected jam has cleared.
        this.clearFault('cylinderext');
      }
    }

    // Buzzer Sound Loop for Active Critical Alarms
    const hasActiveFault = Object.values(state.faults).some(f => f);
    if (hasActiveFault) {
      if (now - this.alarmBuzzerInterval > 1400) {
        audio.playAlarmBuzzer();
        this.alarmBuzzerInterval = now;
      }
    }
  }

  raiseFault(faultKey, code, message) {
    if (!state.faults[faultKey]) {
      state.faults[faultKey] = true;
      audio.playAlarmBuzzer();
      this.logEvent(code, 'danger', message);
      this.updateUIFaultState(faultKey, true);
    }
  }

  clearFault(faultKey) {
    if (state.faults[faultKey]) {
      state.faults[faultKey] = false;
      this.logEvent(`CLR-${faultKey.toUpperCase()}`, 'info', `Fault condition for ${faultKey} has resolved.`);
      this.updateUIFaultState(faultKey, false);
    }
  }

  logEvent(code, type, msg) {
    const entry = {
      time: new Date().toLocaleTimeString(),
      code,
      type,
      msg
    };
    state.alarmLog.unshift(entry);
    if (state.alarmLog.length > 50) state.alarmLog.pop();
    renderAlarmLog();
  }

  updateUIFaultState(key, isActive) {
    const card = document.getElementById(`fdd-${key}`);
    const pill = document.getElementById(`pill-${key}`);
    const faultBtn = document.getElementById(`inject-${key.replace('airleak', 'leak').replace('lowpressure', 'lowpress').replace('cylinderext', 'jam').replace('sortingmismatch', 'miss').replace('sensorfault', 'sensor')}-btn`);

    if (card && pill) {
      if (isActive) {
        card.classList.add('active-fault');
        pill.classList.remove('pill-ok');
        pill.classList.add('pill-alert');
        pill.textContent = 'FAULT';
      } else {
        card.classList.remove('active-fault');
        pill.classList.remove('pill-alert');
        pill.classList.add('pill-ok');
        pill.textContent = 'OK';
      }
    }

    if (faultBtn) {
      faultBtn.classList.toggle('active', isActive);
    }

    updateOverallSystemStatus();
  }
}

const fdd = new FaultDiagnosticEngine();

// ============================================================================
// SIMULATION & PNEUMATIC KINEMATICS ENGINE
// ============================================================================
class PneumaticSimulationEngine {
  constructor() {
    this.lastTime = performance.now();
  }

  step(now) {
    const dt = Math.min((now - this.lastTime) / 1000, 0.1);
    this.lastTime = now;

    if (!state.isRunning || state.isEmergencyStop) {
      return;
    }

    // Drop back to simulation if the bridge has gone quiet for 3 seconds.
    if (state.live.connected && now - state.live.lastPacket > 3000) {
      setPlcLinkState(false, 'PLC: WS TIMEOUT (SIM ACTIVE)');
      fdd.logEvent('WS-TIMEOUT', 'warn', 'No telemetry from PLC bridge for 3s. Reverting to internal simulation.');
    }

    const live = isLiveTelemetry();

    // 1. Pressure Physics Model
    // Skipped on a live link: actualPressure is written straight from the
    // PLC's analog input by ingestPlcTelemetry().
    if (!live) {
      if (state.faults.airleak) {
        // Continuous pressure decay
        state.actualPressure = Math.max(1.8, state.actualPressure - (0.28 * dt));
      } else {
        // Regulate back towards nominal supply pressure
        const targetP = state.nominalSupplyPressure;
        state.actualPressure += (targetP - state.actualPressure) * (5.0 * dt);
      }

      // Add sensor noise jitter
      const noise = (Math.random() - 0.5) * 0.03;
      state.actualPressure = Math.max(0.1, state.actualPressure + noise);
    }

    // 2. Specimen Feeder
    // On a live link specimens are spawned from the physical infeed IR sensor's
    // rising edge instead of on a fixed timer.
    if (!live) {
      state.feedTimer += dt;
      if (state.feedTimer >= state.feedIntervalSec) {
        state.feedTimer = 0;
        // 55% chance of Metal, 45% Non-Metal
        const isMetal = Math.random() < 0.55;
        this.spawnSpecimen(isMetal ? 'metal' : 'nonmetal');
      }
    }

    // 3. Move Specimens & Sensor Interactions
    this.updateSpecimens(dt);

    // 4. Cylinder Kinematics
    this.updateCylinder(dt);

    // 5. Run FDD Check
    fdd.update(now, dt);

    // 6. Record Waveform History
    state.pressureHistory.push(state.actualPressure);
    if (state.pressureHistory.length > 120) {
      state.pressureHistory.shift();
    }
  }

  spawnSpecimen(type) {
    const specimen = {
      id: state.nextSpecimenId++,
      type, // 'metal' or 'nonmetal'
      x: BELT_LAYOUT.spawnX, // infeed position
      y: 180,
      width: 32,
      height: 32,
      detectedByInductive: false,
      pushed: false,
      pushOffsetY: 0,
      binned: null,
      passedSensor: false,
      classified: false
    };
    state.specimens.push(specimen);
  }

  updateSpecimens(dt) {
    // Sensor interaction zones (Canvas coordinates):
    // Infeed IR Sensor: x = 110
    // Inductive Proximity Sensor: x = 400
    // Janatics Cylinder Pusher: x = 580
    // Bin 1 (Metal): y >= 280 at x = 580
    // Bin 2 (Non-Metal): x >= 1060

    const live = isLiveTelemetry();
    const beltSpeed = state.conveyorRunning ? state.conveyorSpeed * dt : 0;
    let inductiveActive = state.faults.sensorfault; // If sensor stuck fault, keep HIGH!
    let infeedActive = false;

    for (let i = state.specimens.length - 1; i >= 0; i--) {
      const sp = state.specimens[i];

      // If already pushed into Bin 1 chute
      if (sp.binned === 'bin1') {
        sp.y += 140 * dt;
        if (sp.y > 380) {
          state.specimens.splice(i, 1);
        }
        continue;
      }

      // If dropped in Bin 2 (end of line)
      if (sp.binned === 'bin2') {
        sp.y += 120 * dt;
        if (sp.y > 380) {
          state.specimens.splice(i, 1);
        }
        continue;
      }

      // Move along belt
      sp.x += beltSpeed;

      // Infeed IR Sensor Detection
      if (Math.abs(sp.x - BELT_LAYOUT.infeedIR.x) < BELT_LAYOUT.infeedIR.halfWidth) {
        infeedActive = true;
      }

      // On a live link the piece's material is unknown until the physical
      // inductive sensor asserts while the piece is inside its sensing window.
      if (live && !sp.classified && inInductiveWindow(sp.x)) {
        if (state.sensors.inductive) {
          sp.type = 'metal';
          sp.classified = true;
        }
      }

      // Inductive Sensor Detection Zone
      if (inInductiveWindow(sp.x)) {
        if (sp.type === 'metal') {
          inductiveActive = true;
          if (!sp.detectedByInductive) {
            sp.detectedByInductive = true;
            audio.playMetalBeep();
            fdd.logEvent('IND-001', 'info', `Inductive Proximity Sensor detected Metallic Specimen #${sp.id}.`);
          }
        }
      }

      // Pre-trigger Solenoid as metal approaches pusher (x >= 520)
      // On a live link the PLC owns the valve - never command it from here.
      if (!live && sp.type === 'metal' && sp.detectedByInductive && sp.x >= BELT_LAYOUT.pusher.preTriggerStart && sp.x <= BELT_LAYOUT.pusher.preTriggerEnd && !sp.pushed) {
        if (!state.faults.cylinderext && state.actualPressure >= state.pressureThresholdMin) {
          this.triggerCylinder();
        } else if (state.faults.cylinderext || state.actualPressure < state.pressureThresholdMin) {
          fdd.raiseFault('sortingmismatch', 'ERR-04: SORT MISMATCH', `Specimen #${sp.id} (Metal) missed: low pressure or cylinder stuck!`);
        }
      }

      // Pusher Blade Contact & Deflection Zone
      if (sp.x >= BELT_LAYOUT.pusher.zoneStart && sp.x <= BELT_LAYOUT.pusher.zoneEnd && !sp.pushed) {
        if (sp.detectedByInductive) {
          if (state.cylinderPos > 0.25 && !state.faults.cylinderext && state.actualPressure >= state.pressureThresholdMin) {
            sp.pushed = true;
            sp.binned = 'bin1';
            state.counts.metal++;
            state.counts.total++;
            this.pulseBinSensor('irBin1');
            updateKPICounters();
            fdd.logEvent('SORT-001', 'info', `Specimen #${sp.id} (Metal) successfully deflected into Bin 1.`);
          }
        }
      }

      // End of Belt -> Bin 2 (Non-Metal)
      if (sp.x >= BELT_LAYOUT.bin2DropX && !sp.binned) {
        sp.binned = 'bin2';
        if (sp.type === 'nonmetal') {
          state.counts.nonmetal++;
          state.counts.total++;
          this.pulseBinSensor('irBin2');
        } else {
          // A metal passed through without being sorted!
          state.counts.missed++;
          fdd.raiseFault('sortingmismatch', 'ERR-04: SORTING MISS', `Metallic specimen #${sp.id} escaped to Non-Metal Bin 2!`);
        }
        updateKPICounters();
      }
    }

    if (!live) {
      state.sensors.inductive = inductiveActive;
      state.sensors.irInfeed = infeedActive;
    }
  }

  triggerCylinder() {
    if (state.solenoidEnergized) return;

    state.solenoidEnergized = true;
    state.cylinderTarget = 1.0;
    state.cylinderActuations++;
    state.lastStrokeStart = performance.now();
    audio.playPneumaticHiss();

    // Pneumatic pressure drop during stroke (slight realistic ripple)
    state.actualPressure = Math.max(5.2, state.actualPressure - 0.12);

    // Auto retract after 180ms
    setTimeout(() => {
      state.cylinderTarget = 0.0;
      setTimeout(() => {
        state.solenoidEnergized = false;
        audio.playPneumaticHiss();
      }, 160);
    }, 180);
  }

  updateCylinder(dt) {
    if (state.injected.cylinderext) {
      // Cylinder is mechanically stuck! Stalls at 0.15
      state.cylinderPos = 0.15;
      state.cylinderStrokeTimeMs = 450;
      return;
    }

    // On a live link the physical 5/2 valve decides extend vs. retract; the
    // kinematics below just animate towards whatever the coil is commanding.
    if (isLiveTelemetry()) {
      state.cylinderTarget = state.solenoidEnergized ? 1.0 : 0.0;
    }

    // Stroke speed depends on pneumatic pressure
    // Normal: 6 bar -> ~115ms
    // Low: 3 bar -> ~320ms
    const speedMultiplier = Math.max(0.3, state.actualPressure / 6.0);
    const speed = 7.5 * speedMultiplier; // units/sec

    if (state.cylinderPos < state.cylinderTarget) {
      state.cylinderPos = Math.min(state.cylinderTarget, state.cylinderPos + speed * dt);
      state.cylinderStrokeTimeMs = Math.round(115 / Math.max(0.4, speedMultiplier));
    } else if (state.cylinderPos > state.cylinderTarget) {
      state.cylinderPos = Math.max(state.cylinderTarget, state.cylinderPos - speed * dt);
    }
  }

  pulseBinSensor(sensorKey) {
    state.sensors[sensorKey] = true;
    setTimeout(() => {
      state.sensors[sensorKey] = false;
    }, 350);
  }
}

const sim = new PneumaticSimulationEngine();

// ============================================================================
// CANVAS DIGITAL TWIN RENDERER
// ============================================================================
const dtCanvas = document.getElementById('digitalTwinCanvas');
const dtCtx = dtCanvas.getContext('2d');

// ----------------------------------------------------------------------------
// HiDPI CANVAS SIZING
// ----------------------------------------------------------------------------
// A canvas has two sizes: its CSS box and its drawing buffer. If the buffer is
// left at the HTML width/height attributes, the browser stretches it to fit,
// which both blurs the output on a Retina display (devicePixelRatio 2) and
// distorts it whenever the two aspect ratios disagree. These helpers size the
// buffer to real device pixels and hand the drawing code a stable logical
// coordinate space to keep using.

// Element sizes are cached and refreshed by a ResizeObserver rather than
// measured each frame: reading layout geometry inside the 60 FPS render loop,
// right after the UI pass has written to the DOM, forces a synchronous reflow.
const canvasSizes = new WeakMap();

function observeCanvasSize(canvas) {
  canvasSizes.set(canvas, { w: canvas.clientWidth, h: canvas.clientHeight });
  const ro = new ResizeObserver((entries) => {
    for (const entry of entries) {
      const box = entry.contentBoxSize && entry.contentBoxSize[0];
      canvasSizes.set(canvas, box
        ? { w: box.inlineSize, h: box.blockSize }
        : { w: entry.contentRect.width, h: entry.contentRect.height });
    }
  });
  ro.observe(canvas);
}

/**
 * Prepares a canvas for drawing at native device resolution.
 *
 * With logicalW/logicalH, the drawing space stays that fixed size and is scaled
 * uniformly to fit ("contain"), so circles stay circular no matter how the
 * panel is resized. Without them, the drawing space is simply CSS pixels.
 *
 * Returns the logical dimensions to draw against, plus the raw buffer size for
 * anything that needs to paint edge to edge.
 */
function fitCanvas(canvas, ctx, logicalW, logicalH) {
  const dpr = window.devicePixelRatio || 1;
  const size = canvasSizes.get(canvas) || { w: canvas.clientWidth, h: canvas.clientHeight };
  const pxW = Math.max(1, Math.round(size.w * dpr));
  const pxH = Math.max(1, Math.round(size.h * dpr));

  // Assigning width/height clears the canvas and resets its context state, so
  // only do it when the size actually changed.
  if (canvas.width !== pxW || canvas.height !== pxH) {
    canvas.width = pxW;
    canvas.height = pxH;
  }

  if (logicalW && logicalH) {
    const scale = Math.min(pxW / logicalW, pxH / logicalH);
    ctx.setTransform(scale, 0, 0, scale, (pxW - logicalW * scale) / 2, (pxH - logicalH * scale) / 2);
    return { w: logicalW, h: logicalH, pxW, pxH };
  }

  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { w: size.w, h: size.h, pxW, pxH };
}

// The schematic is authored against this fixed coordinate space.
const TWIN_LOGICAL_W = 1180;
const TWIN_LOGICAL_H = 360;

let beltOffset = 0;

function renderDigitalTwin() {
  const { w, h, pxW, pxH } = fitCanvas(dtCanvas, dtCtx, TWIN_LOGICAL_W, TWIN_LOGICAL_H);

  // Background, painted untransformed so it also covers any letterboxing left
  // over when the panel's aspect ratio does not match the schematic's.
  dtCtx.save();
  dtCtx.setTransform(1, 0, 0, 1, 0, 0);
  dtCtx.fillStyle = '#0a0d16';
  dtCtx.fillRect(0, 0, pxW, pxH);
  dtCtx.restore();

  // Grid blueprint lines
  dtCtx.strokeStyle = 'rgba(255, 255, 255, 0.025)';
  dtCtx.lineWidth = 1;
  for (let x = 0; x < w; x += 40) {
    dtCtx.beginPath();
    dtCtx.moveTo(x, 0);
    dtCtx.lineTo(x, h);
    dtCtx.stroke();
  }
  for (let y = 0; y < h; y += 40) {
    dtCtx.beginPath();
    dtCtx.moveTo(0, y);
    dtCtx.lineTo(w, y);
    dtCtx.stroke();
  }

  // 1. Conveyor Belt Assembly (x: 20 to 1050, y: 160 to 220)
  const beltY = BELT_LAYOUT.beltY;
  const beltH = BELT_LAYOUT.beltH;
  const beltStartX = BELT_LAYOUT.beltStartX;
  const beltEndX = BELT_LAYOUT.beltEndX;

  // Conveyor Frame / Bed
  dtCtx.fillStyle = '#1e293b';
  dtCtx.fillRect(beltStartX - 5, beltY - 6, beltEndX - beltStartX + 10, beltH + 12);
  dtCtx.strokeStyle = '#334155';
  dtCtx.lineWidth = 2;
  dtCtx.strokeRect(beltStartX - 5, beltY - 6, beltEndX - beltStartX + 10, beltH + 12);

  // Rotating Drive Rollers (Idler & Drive)
  renderRoller(beltStartX + 15, beltY + beltH / 2, 22);
  renderRoller(beltEndX - 15, beltY + beltH / 2, 22);

  // Belt surface with moving texture
  dtCtx.fillStyle = '#0f172a';
  dtCtx.fillRect(beltStartX + 15, beltY, beltEndX - beltStartX - 30, beltH);

  if (state.conveyorRunning) {
    beltOffset = (beltOffset + 2) % 20;
  }

  dtCtx.strokeStyle = '#1e293b';
  dtCtx.lineWidth = 2;
  for (let bx = beltStartX + 20 - beltOffset; bx < beltEndX - 20; bx += 20) {
    if (bx > beltStartX + 15) {
      dtCtx.beginPath();
      dtCtx.moveTo(bx, beltY);
      dtCtx.lineTo(bx, beltY + beltH);
      dtCtx.stroke();
    }
  }

  // Conveyor Direction Arrows
  dtCtx.fillStyle = 'rgba(56, 189, 248, 0.2)';
  for (let ax = beltStartX + 60; ax < beltEndX - 80; ax += 180) {
    drawArrow(dtCtx, ax, beltY + beltH / 2, ax + 25, beltY + beltH / 2, 6);
  }

  // 2. Infeed IR Sensor
  renderIRSensor(BELT_LAYOUT.infeedIR.x, beltY - 35, state.sensors.irInfeed, 'INFEED IR');

  // 3. Inductive Proximity Sensor (8mm NPN-NO)
  renderInductiveSensor(BELT_LAYOUT.inductive.x, beltY - 55, state.sensors.inductive);

  // 4. 5/2 Single Solenoid Valve & Air Routing
  renderSolenoidValve(BELT_LAYOUT.pusher.x, 25, state.solenoidEnergized);

  // 5. Janatics Double Acting Cylinder (pushing DOWN across the belt)
  renderPneumaticCylinder(BELT_LAYOUT.pusher.x, beltY - 80, state.cylinderPos);

  // 6. Collection Bins
  // Bin 1: Metal Bin (below the pusher)
  renderCollectionBin(BELT_LAYOUT.bin1.x, BELT_LAYOUT.bin1.y, BELT_LAYOUT.bin1.w, BELT_LAYOUT.bin1.h, 'BIN 1 (METAL)', '#38bdf8', state.counts.metal, state.sensors.irBin1);
  // Bin 2: Non-Metal Bin (at end of belt)
  renderCollectionBin(BELT_LAYOUT.bin2.x, BELT_LAYOUT.bin2.y, BELT_LAYOUT.bin2.w, BELT_LAYOUT.bin2.h, 'BIN 2 (NON-METAL)', '#f59e0b', state.counts.nonmetal, state.sensors.irBin2);

  // 7. Render Moving Specimens on Conveyor
  for (const sp of state.specimens) {
    renderSpecimen(sp);
  }
}

function renderRoller(cx, cy, r) {
  dtCtx.save();
  dtCtx.beginPath();
  dtCtx.arc(cx, cy, r, 0, Math.PI * 2);
  dtCtx.fillStyle = '#475569';
  dtCtx.fill();
  dtCtx.strokeStyle = '#94a3b8';
  dtCtx.lineWidth = 2;
  dtCtx.stroke();

  // Spoke indicator
  const angle = (beltOffset / 20) * Math.PI * 2;
  dtCtx.beginPath();
  dtCtx.moveTo(cx, cy);
  dtCtx.lineTo(cx + Math.cos(angle) * (r - 4), cy + Math.sin(angle) * (r - 4));
  dtCtx.strokeStyle = '#cbd5e1';
  dtCtx.lineWidth = 3;
  dtCtx.stroke();
  dtCtx.restore();
}

function renderIRSensor(x, y, isActive, label) {
  dtCtx.save();
  // Bracket
  dtCtx.fillStyle = '#334155';
  dtCtx.fillRect(x - 12, y, 24, 25);
  dtCtx.strokeStyle = '#64748b';
  dtCtx.strokeRect(x - 12, y, 24, 25);

  // IR Emitter & Receiver Lenses
  dtCtx.fillStyle = isActive ? '#ef4444' : '#7f1d1d';
  dtCtx.beginPath();
  dtCtx.arc(x - 5, y + 25, 4, 0, Math.PI * 2);
  dtCtx.fill();

  dtCtx.fillStyle = isActive ? '#38bdf8' : '#0369a1';
  dtCtx.beginPath();
  dtCtx.arc(x + 5, y + 25, 4, 0, Math.PI * 2);
  dtCtx.fill();

  // Optical Beam down to belt
  if (isActive) {
    dtCtx.strokeStyle = 'rgba(239, 68, 68, 0.7)';
    dtCtx.lineWidth = 2;
    dtCtx.setLineDash([4, 4]);
    dtCtx.beginPath();
    dtCtx.moveTo(x, y + 25);
    dtCtx.lineTo(x, y + 70);
    dtCtx.stroke();
    dtCtx.setLineDash([]);
  }

  // Label
  dtCtx.fillStyle = '#94a3b8';
  dtCtx.font = '9px JetBrains Mono';
  dtCtx.textAlign = 'center';
  dtCtx.fillText(label, x, y - 4);
  dtCtx.restore();
}

function renderInductiveSensor(x, y, isActive) {
  dtCtx.save();
  // Cylindrical Sensor Body (6-36V M12/M18 Inductive)
  const grad = dtCtx.createLinearGradient(x - 10, y, x + 10, y);
  grad.addColorStop(0, '#64748b');
  grad.addColorStop(0.5, '#cbd5e1');
  grad.addColorStop(1, '#475569');

  dtCtx.fillStyle = grad;
  dtCtx.fillRect(x - 10, y, 20, 48);
  dtCtx.strokeStyle = '#1e293b';
  dtCtx.strokeRect(x - 10, y, 20, 48);

  // Sensing Face (Blue Plastic Cap)
  dtCtx.fillStyle = isActive ? '#38bdf8' : '#0284c7';
  dtCtx.fillRect(x - 10, y + 48, 20, 6);

  // Active Magnetic Field Lines
  if (isActive) {
    dtCtx.strokeStyle = 'rgba(168, 85, 247, 0.8)';
    dtCtx.lineWidth = 1.5;
    for (let r = 8; r <= 20; r += 6) {
      dtCtx.beginPath();
      dtCtx.arc(x, y + 54, r, 0, Math.PI);
      dtCtx.stroke();
    }

    // Status LED on sensor
    dtCtx.fillStyle = '#ef4444';
    dtCtx.shadowColor = '#ef4444';
    dtCtx.shadowBlur = 8;
    dtCtx.beginPath();
    dtCtx.arc(x, y + 12, 3, 0, Math.PI * 2);
    dtCtx.fill();
    dtCtx.shadowBlur = 0;
  }

  // Label
  dtCtx.fillStyle = '#a855f7';
  dtCtx.font = '10px JetBrains Mono';
  dtCtx.textAlign = 'center';
  dtCtx.fillText('INDUCTIVE (8mm)', x, y - 6);
  dtCtx.restore();
}

function renderSolenoidValve(x, y, isEnergized) {
  dtCtx.save();
  // Valve Manifold Body
  dtCtx.fillStyle = '#1e293b';
  dtCtx.strokeStyle = isEnergized ? '#38bdf8' : '#475569';
  dtCtx.lineWidth = 1.5;
  dtCtx.fillRect(x - 35, y, 70, 32);
  dtCtx.strokeRect(x - 35, y, 70, 32);

  // 12V/24V Solenoid Coil on side
  dtCtx.fillStyle = isEnergized ? '#2563eb' : '#0f172a';
  dtCtx.fillRect(x + 35, y + 4, 18, 24);
  dtCtx.strokeStyle = '#38bdf8';
  dtCtx.strokeRect(x + 35, y + 4, 18, 24);

  // Coil LED
  dtCtx.fillStyle = isEnergized ? '#34d399' : '#334155';
  dtCtx.beginPath();
  dtCtx.arc(x + 44, y + 16, 3, 0, Math.PI * 2);
  dtCtx.fill();

  // Valve schematic icon inside
  dtCtx.fillStyle = '#94a3b8';
  dtCtx.font = '9px JetBrains Mono';
  dtCtx.textAlign = 'center';
  dtCtx.fillText('5/2 VALVE', x, y + 15);
  dtCtx.fillText(isEnergized ? '1→4 EXT' : '1→2 RET', x, y + 26);

  // Air tube to cylinder
  dtCtx.strokeStyle = isEnergized ? '#38bdf8' : '#334155';
  dtCtx.lineWidth = 2.5;
  dtCtx.beginPath();
  dtCtx.moveTo(x - 10, y + 32);
  dtCtx.lineTo(x - 10, y + 55);
  dtCtx.stroke();

  dtCtx.strokeStyle = !isEnergized ? '#38bdf8' : '#334155';
  dtCtx.beginPath();
  dtCtx.moveTo(x + 10, y + 32);
  dtCtx.lineTo(x + 10, y + 55);
  dtCtx.stroke();

  dtCtx.restore();
}

function renderPneumaticCylinder(x, y, strokePos) {
  dtCtx.save();
  const cylW = 34;
  const cylH = 65;

  // Aluminum Barrel (Janatics 20mm Bore)
  const grad = dtCtx.createLinearGradient(x - cylW / 2, y, x + cylW / 2, y);
  grad.addColorStop(0, '#475569');
  grad.addColorStop(0.5, '#94a3b8');
  grad.addColorStop(1, '#334155');

  dtCtx.fillStyle = grad;
  dtCtx.fillRect(x - cylW / 2, y, cylW, cylH);
  dtCtx.strokeStyle = '#cbd5e1';
  dtCtx.lineWidth = 1.5;
  dtCtx.strokeRect(x - cylW / 2, y, cylW, cylH);

  // Tie-rods
  dtCtx.strokeStyle = '#1e293b';
  dtCtx.lineWidth = 2;
  dtCtx.strokeRect(x - cylW / 2 + 2, y, cylW - 4, cylH);

  // Janatics Label
  dtCtx.fillStyle = '#0f172a';
  dtCtx.font = '8px Rajdhani';
  dtCtx.textAlign = 'center';
  dtCtx.fillText('JANATICS 20x50', x, y + 35);

  // Piston Rod & Pusher Head (Extending DOWN towards belt)
  const maxStrokePx = 70;
  const currentStroke = strokePos * maxStrokePx;

  // Steel Piston Rod (10mm dia)
  dtCtx.fillStyle = '#e2e8f0';
  dtCtx.fillRect(x - 4, y + cylH, 8, currentStroke);
  dtCtx.strokeStyle = '#64748b';
  dtCtx.strokeRect(x - 4, y + cylH, 8, currentStroke);

  // Pusher Blade (Pushes specimen into Bin 1)
  const bladeY = y + cylH + currentStroke;
  dtCtx.fillStyle = '#38bdf8';
  dtCtx.fillRect(x - 22, bladeY, 44, 8);
  dtCtx.strokeStyle = '#fff';
  dtCtx.strokeRect(x - 22, bladeY, 44, 8);

  dtCtx.restore();
}

function renderCollectionBin(x, y, w, h, title, accentColor, count, sensorActive) {
  dtCtx.save();
  // Bin Chute Frame
  dtCtx.fillStyle = 'rgba(15, 23, 42, 0.9)';
  dtCtx.strokeStyle = accentColor;
  dtCtx.lineWidth = 2;
  dtCtx.fillRect(x, y, w, h);
  dtCtx.strokeRect(x, y, w, h);

  // Chute entrance gradient
  const grad = dtCtx.createLinearGradient(x, y, x, y + 25);
  grad.addColorStop(0, 'rgba(255, 255, 255, 0.1)');
  grad.addColorStop(1, 'transparent');
  dtCtx.fillStyle = grad;
  dtCtx.fillRect(x, y, w, 25);

  // Sensor verification beam indicator
  if (sensorActive) {
    dtCtx.fillStyle = 'rgba(16, 185, 129, 0.35)';
    dtCtx.fillRect(x + 5, y + 10, w - 10, 15);
  }

  // Bin Header
  dtCtx.fillStyle = accentColor;
  dtCtx.font = '10px Rajdhani';
  dtCtx.textAlign = 'center';
  dtCtx.fillText(title, x + w / 2, y + 16);

  // Count inside bin
  dtCtx.fillStyle = '#fff';
  dtCtx.font = '22px JetBrains Mono';
  dtCtx.fillText(count.toString(), x + w / 2, y + 55);

  dtCtx.fillStyle = '#94a3b8';
  dtCtx.font = '9px Inter';
  dtCtx.fillText('ITEMS SORTED', x + w / 2, y + 74);

  // Verification Sensor Tag
  dtCtx.fillStyle = sensorActive ? '#34d399' : '#64748b';
  dtCtx.font = '8px JetBrains Mono';
  dtCtx.fillText(sensorActive ? 'VERIFIED ✓' : 'IR STANDBY', x + w / 2, y + 94);

  dtCtx.restore();
}

function renderSpecimen(sp) {
  dtCtx.save();
  const isMetal = sp.type === 'metal';

  if (isMetal) {
    // Metallic Stainless Steel Cylinder with shine & reflection
    const grad = dtCtx.createLinearGradient(sp.x - 16, sp.y - 16, sp.x + 16, sp.y + 16);
    grad.addColorStop(0, '#94a3b8');
    grad.addColorStop(0.3, '#f1f5f9');
    grad.addColorStop(0.7, '#64748b');
    grad.addColorStop(1, '#334155');

    dtCtx.fillStyle = grad;
    dtCtx.beginPath();
    dtCtx.arc(sp.x, sp.y, 14, 0, Math.PI * 2);
    dtCtx.fill();
    dtCtx.strokeStyle = '#38bdf8';
    dtCtx.lineWidth = 1.5;
    dtCtx.stroke();

    // Specimen ID
    dtCtx.fillStyle = '#0f172a';
    dtCtx.font = '9px JetBrains Mono';
    dtCtx.textAlign = 'center';
    dtCtx.fillText(`M${sp.id}`, sp.x, sp.y + 3);
  } else {
    // Non-Metallic Specimen (Yellow Polymer/Wood block)
    dtCtx.fillStyle = '#f59e0b';
    dtCtx.fillRect(sp.x - 13, sp.y - 13, 26, 26);
    dtCtx.strokeStyle = '#b45309';
    dtCtx.lineWidth = 1.5;
    dtCtx.strokeRect(sp.x - 13, sp.y - 13, 26, 26);

    // Diagonal texture
    dtCtx.strokeStyle = 'rgba(0, 0, 0, 0.2)';
    dtCtx.beginPath();
    dtCtx.moveTo(sp.x - 13, sp.y - 13);
    dtCtx.lineTo(sp.x + 13, sp.y + 13);
    dtCtx.stroke();

    // Specimen ID
    dtCtx.fillStyle = '#000';
    dtCtx.font = '9px JetBrains Mono';
    dtCtx.textAlign = 'center';
    dtCtx.fillText(`N${sp.id}`, sp.x, sp.y + 3);
  }

  dtCtx.restore();
}

function drawArrow(ctx, fromX, fromY, toX, toY, headLength) {
  const dx = toX - fromX;
  const dy = toY - fromY;
  const angle = Math.atan2(dy, dx);
  ctx.beginPath();
  ctx.moveTo(fromX, fromY);
  ctx.lineTo(toX, toY);
  ctx.lineTo(toX - headLength * Math.cos(angle - Math.PI / 6), toY - headLength * Math.sin(angle - Math.PI / 6));
  ctx.moveTo(toX, toY);
  ctx.lineTo(toX - headLength * Math.cos(angle + Math.PI / 6), toY - headLength * Math.sin(angle + Math.PI / 6));
  ctx.stroke();
}

// ============================================================================
// PRESSURE WAVEFORM CANVAS RENDERER
// ============================================================================
const waveCanvas = document.getElementById('pressureWaveformCanvas');
const waveCtx = waveCanvas.getContext('2d');

observeCanvasSize(dtCanvas);
observeCanvasSize(waveCanvas);

function renderPressureWaveform() {
  // The waveform has no fixed authored size - it simply fills its box.
  const { w, h } = fitCanvas(waveCanvas, waveCtx);

  waveCtx.fillStyle = '#090d16';
  waveCtx.fillRect(0, 0, w, h);

  // Grid lines
  waveCtx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
  waveCtx.lineWidth = 1;
  const maxP = 9.0;
  const minP = 1.0;

  // Pressure ticks: 2, 4.5, 6, 8 bar
  const pValues = [2.0, 4.5, 6.0, 8.0];
  waveCtx.fillStyle = '#64748b';
  waveCtx.font = '9px JetBrains Mono';
  waveCtx.textAlign = 'left';

  for (const pv of pValues) {
    const y = h - ((pv - minP) / (maxP - minP)) * h;
    waveCtx.beginPath();
    waveCtx.moveTo(0, y);
    waveCtx.lineTo(w, y);
    waveCtx.stroke();
    waveCtx.fillText(`${pv} bar`, 6, y - 2);
  }

  // Draw 4.5 bar Threshold cutoff line (Red dashed)
  const threshY = h - ((state.pressureThresholdMin - minP) / (maxP - minP)) * h;
  waveCtx.strokeStyle = 'rgba(239, 68, 68, 0.7)';
  waveCtx.setLineDash([4, 4]);
  waveCtx.beginPath();
  waveCtx.moveTo(0, threshY);
  waveCtx.lineTo(w, threshY);
  waveCtx.stroke();
  waveCtx.setLineDash([]);

  // Plot Rolling Waveform
  const points = state.pressureHistory;
  const stepX = w / (points.length - 1);

  // Gradient fill under curve
  const grad = waveCtx.createLinearGradient(0, 0, 0, h);
  if (state.actualPressure < state.pressureThresholdMin || state.faults.airleak) {
    grad.addColorStop(0, 'rgba(239, 68, 68, 0.35)');
    grad.addColorStop(1, 'rgba(239, 68, 68, 0.0)');
  } else {
    grad.addColorStop(0, 'rgba(56, 189, 248, 0.35)');
    grad.addColorStop(1, 'rgba(56, 189, 248, 0.0)');
  }

  waveCtx.beginPath();
  for (let i = 0; i < points.length; i++) {
    const val = points[i];
    const px = i * stepX;
    const py = h - ((val - minP) / (maxP - minP)) * h;
    if (i === 0) waveCtx.moveTo(px, py);
    else waveCtx.lineTo(px, py);
  }

  waveCtx.lineTo(w, h);
  waveCtx.lineTo(0, h);
  waveCtx.closePath();
  waveCtx.fillStyle = grad;
  waveCtx.fill();

  // Draw stroke line
  waveCtx.beginPath();
  for (let i = 0; i < points.length; i++) {
    const val = points[i];
    const px = i * stepX;
    const py = h - ((val - minP) / (maxP - minP)) * h;
    if (i === 0) waveCtx.moveTo(px, py);
    else waveCtx.lineTo(px, py);
  }

  waveCtx.strokeStyle = (state.actualPressure < state.pressureThresholdMin) ? '#ef4444' : '#38bdf8';
  waveCtx.lineWidth = 2;
  waveCtx.stroke();

  // Current value dot
  const lastVal = points[points.length - 1];
  const lastX = w;
  const lastY = h - ((lastVal - minP) / (maxP - minP)) * h;

  waveCtx.fillStyle = '#fff';
  waveCtx.beginPath();
  waveCtx.arc(lastX - 2, lastY, 4, 0, Math.PI * 2);
  waveCtx.fill();
}

// ============================================================================
// UI DOM UPDATES & TELEMETRY SYNC
// ============================================================================
function updateKPICounters() {
  const metalEl = document.getElementById('kpi-metal-val');
  const nonmetalEl = document.getElementById('kpi-nonmetal-val');
  const totalEl = document.getElementById('kpi-total-val');
  const solenoidEl = document.getElementById('kpi-solenoid-actuations');
  const metalPctEl = document.getElementById('kpi-metal-pct');
  const nonmetalPctEl = document.getElementById('kpi-nonmetal-pct');
  const metalBar = document.getElementById('metal-bar-fill');
  const nonmetalBar = document.getElementById('nonmetal-bar-fill');

  const tot = state.counts.total;
  const m = state.counts.metal;
  const nm = state.counts.nonmetal;

  if (metalEl) metalEl.textContent = m;
  if (nonmetalEl) nonmetalEl.textContent = nm;
  if (totalEl) totalEl.textContent = tot;
  if (solenoidEl) solenoidEl.textContent = state.cylinderActuations;

  const mPct = tot > 0 ? ((m / tot) * 100).toFixed(1) : '0.0';
  const nmPct = tot > 0 ? ((nm / tot) * 100).toFixed(1) : '0.0';

  if (metalPctEl) metalPctEl.textContent = `${mPct}%`;
  if (nonmetalPctEl) nonmetalPctEl.textContent = `${nmPct}%`;
  if (metalBar) metalBar.style.width = `${Math.min(100, Math.max(10, mPct))}%`;
  if (nonmetalBar) nonmetalBar.style.width = `${Math.min(100, Math.max(10, nmPct))}%`;
}

function updateLiveGaugesAndBadges() {
  // Pressure Card
  const pValEl = document.getElementById('kpi-pressure-val');
  const pBadge = document.getElementById('pressure-status-badge');
  const pBar = document.getElementById('pressure-bar-fill');
  const pPsi = document.getElementById('pressure-psi-equiv');
  const gaugeBig = document.getElementById('gauge-big-val');
  const gaugeNeedle = document.getElementById('gauge-needle');

  const p = state.actualPressure;
  const pPsiVal = (p * 14.5038).toFixed(1);

  if (pValEl) pValEl.textContent = p.toFixed(2);
  if (gaugeBig) gaugeBig.textContent = p.toFixed(1);
  if (pPsi) pPsi.textContent = `~${pPsiVal} PSI`;

  if (pBar) {
    const pct = Math.min(100, Math.max(0, (p / 10.0) * 100));
    pBar.style.width = `${pct}%`;
  }

  // Needle angle: 0 bar = -135deg, 10 bar = 45deg (total arc: 180deg)
  if (gaugeNeedle) {
    // 0 to 10 bar mapped to angle
    const angleRad = Math.PI - (Math.min(10, Math.max(0, p)) / 10.0) * Math.PI;
    const nx = 100 - Math.cos(angleRad) * 55;
    const ny = 110 - Math.sin(angleRad) * 55;
    gaugeNeedle.setAttribute('x2', nx.toFixed(1));
    gaugeNeedle.setAttribute('y2', ny.toFixed(1));
  }

  if (p < state.pressureThresholdMin) {
    if (pBadge) {
      pBadge.className = 'kpi-badge badge-danger';
      pBadge.textContent = 'LOW FAULT';
    }
  } else if (p > 7.5) {
    if (pBadge) {
      pBadge.className = 'kpi-badge badge-warn';
      pBadge.textContent = 'HIGH';
    }
  } else {
    if (pBadge) {
      pBadge.className = 'kpi-badge badge-ok';
      pBadge.textContent = 'OPTIMAL';
    }
  }

  // Leakage Card
  const leakValEl = document.getElementById('kpi-leak-val');
  const leakBadge = document.getElementById('leak-status-badge');
  const leakLoss = document.getElementById('leak-airloss-val');
  const leakBar = document.getElementById('leak-bar-fill');

  const lRate = state.pressureDecayRate;
  if (leakValEl) leakValEl.textContent = lRate.toFixed(3);
  if (leakLoss) {
    const lossLiters = (lRate * 28.3).toFixed(1);
    leakLoss.textContent = `~${lossLiters} L/min`;
  }

  if (leakBar) {
    const leakPct = Math.min(100, (lRate / 0.1) * 100);
    leakBar.style.width = `${leakPct}%`;
  }

  if (lRate > 0.05) {
    if (leakBadge) {
      leakBadge.className = 'kpi-badge badge-danger';
      leakBadge.textContent = 'AIR LEAK';
    }
  } else {
    if (leakBadge) {
      leakBadge.className = 'kpi-badge badge-ok';
      leakBadge.textContent = 'NO LEAK';
    }
  }

  // Cylinder Stroke Card
  const strokeVal = document.getElementById('kpi-stroke-val');
  const strokeBadge = document.getElementById('stroke-status-badge');
  const strokeBar = document.getElementById('stroke-bar-fill');

  if (strokeVal) strokeVal.textContent = state.cylinderStrokeTimeMs;
  if (state.faults.cylinderext) {
    if (strokeBadge) {
      strokeBadge.className = 'kpi-badge badge-danger';
      strokeBadge.textContent = 'STUCK';
    }
    if (strokeBar) strokeBar.style.width = '100%';
  } else if (state.cylinderStrokeTimeMs > 220) {
    if (strokeBadge) {
      strokeBadge.className = 'kpi-badge badge-warn';
      strokeBadge.textContent = 'SLUGGISH';
    }
  } else {
    if (strokeBadge) {
      strokeBadge.className = 'kpi-badge badge-ok';
      strokeBadge.textContent = 'HEALTHY';
    }
  }

  // Sensor Indicator Pills
  const indPill = document.getElementById('sensor-ind-pill');
  if (indPill) {
    indPill.classList.toggle('active', state.sensors.inductive);
    indPill.textContent = state.sensors.inductive ? 'IND DETECTED' : 'IND STANDBY';
  }

  // Digital Twin Tags
  const convTag = document.getElementById('conveyor-status-tag');
  const valveTag = document.getElementById('valve-state-tag');
  const cylTag = document.getElementById('cylinder-state-tag');

  if (convTag) {
    convTag.textContent = state.isEmergencyStop ? 'CONVEYOR: ESTOP HALTED' : 'CONVEYOR: RUNNING (65 RPM)';
    convTag.className = state.isEmergencyStop ? 'tag tag-red' : 'tag tag-blue';
  }

  if (valveTag) {
    valveTag.textContent = state.solenoidEnergized ? 'VALVE: PORT 1→4 (EXTENDING)' : 'VALVE: PORT 1→2 (RETRACTED)';
    valveTag.className = state.solenoidEnergized ? 'tag tag-green' : 'tag tag-cyan';
  }

  if (cylTag) {
    const mm = (state.cylinderPos * 50).toFixed(0);
    cylTag.textContent = `CYLINDER: ${mm}mm ${state.cylinderPos > 0.1 ? 'EXTENDED' : 'RETRACTED'}`;
  }

  // Update PLC Table Registers
  updatePLCRegisterTable();
  updateCalibrationUI();
}

function updatePLCRegisterTable() {
  const regP = document.getElementById('reg-pressure');
  const regInd = document.getElementById('reg-inductive');
  const regInfeed = document.getElementById('reg-irinfeed');
  const regBin1 = document.getElementById('reg-irbin1');
  const regSol = document.getElementById('reg-solenoid');
  const regConv = document.getElementById('reg-conveyor');
  const regBuz = document.getElementById('reg-buzzer');

  if (regP) regP.textContent = `${state.actualPressure.toFixed(2)} bar`;
  if (regInd) regInd.textContent = state.sensors.inductive ? 'HIGH (1)' : 'LOW (0)';
  if (regInfeed) regInfeed.textContent = state.sensors.irInfeed ? 'HIGH (1)' : 'LOW (0)';
  if (regBin1) regBin1.textContent = state.sensors.irBin1 ? 'HIGH (1)' : 'LOW (0)';
  if (regSol) regSol.textContent = state.solenoidEnergized ? 'ENERGIZED (1) 24V' : 'DE-ENERGIZED (0)';
  if (regConv) regConv.textContent = state.conveyorRunning && !state.isEmergencyStop ? 'ENERGIZED (1)' : 'OFF (0)';

  const hasFault = Object.values(state.faults).some(f => f);
  if (regBuz) regBuz.textContent = hasFault ? 'ALARM ON (1)' : 'OFF (0)';
}

function updateOverallSystemStatus() {
  const pill = document.getElementById('overall-status-pill');
  const text = document.getElementById('system-status-text');

  if (state.isEmergencyStop) {
    pill.className = 'system-status-indicator status-fault';
    text.textContent = 'EMERGENCY STOPPED';
    return;
  }

  const hasCritical = state.faults.airleak || state.faults.lowpressure || state.faults.cylinderext;
  const hasWarning = state.faults.sortingmismatch || state.faults.sensorfault;

  if (hasCritical) {
    pill.className = 'system-status-indicator status-fault';
    text.textContent = 'CRITICAL FAULT DETECTED';
  } else if (hasWarning) {
    pill.className = 'system-status-indicator status-warn';
    text.textContent = 'SYSTEM WARNING';
  } else {
    pill.className = 'system-status-indicator';
    text.textContent = 'SYSTEM NORMAL (OPTIMAL)';
  }
}

function renderAlarmLog() {
  const listEl = document.getElementById('fault-log-list');
  const countBadge = document.getElementById('log-count-badge');
  if (!listEl) return;

  const activeCount = Object.values(state.faults).filter(Boolean).length;
  if (countBadge) countBadge.textContent = `${activeCount} Active`;

  listEl.innerHTML = '';
  for (const item of state.alarmLog.slice(0, 15)) {
    const entry = document.createElement('div');
    entry.className = `log-entry log-${item.type}`;
    entry.innerHTML = `
      <span class="log-time">[${item.time}]</span>
      <span class="log-code">${item.code}</span>
      <span class="log-msg">${item.msg}</span>
    `;
    listEl.appendChild(entry);
  }
}

// ============================================================================
// EVENT LISTENERS & USER INTERACTION
// ============================================================================
function setupEventListeners() {
  // Plain-English explanation layer. Defaults to on so a non-technical viewer
  // is not left guessing; a technical audience can switch it off for the dense
  // SCADA view.
  const explainBtn = document.getElementById('explain-toggle-btn');
  const explainLabel = document.getElementById('explain-label');
  document.body.classList.add('explain-on');
  if (explainBtn) {
    explainBtn.addEventListener('click', () => {
      const on = document.body.classList.toggle('explain-on');
      explainBtn.classList.toggle('active', on);
      if (explainLabel) explainLabel.textContent = on ? 'Explain: ON' : 'Explain: OFF';
    });
  }

  // Audio Mute/Unmute
  const audioBtn = document.getElementById('audio-toggle-btn');
  const audioIcon = document.getElementById('audio-icon');
  const audioLabel = document.getElementById('audio-label');
  if (audioBtn) {
    audioBtn.addEventListener('click', () => {
      const isSoundOn = audio.toggle();
      audioIcon.textContent = isSoundOn ? '🔊' : '🔇';
      audioLabel.textContent = isSoundOn ? 'Buzzer ON' : 'Buzzer OFF';
      audioBtn.classList.toggle('muted', !isSoundOn);
    });
  }

  // Emergency Stop Button
  const estopBtn = document.getElementById('estop-btn');
  if (estopBtn) {
    estopBtn.addEventListener('click', () => {
      state.isEmergencyStop = !state.isEmergencyStop;
      estopBtn.classList.toggle('active', state.isEmergencyStop);
      estopBtn.querySelector('.estop-label').textContent = state.isEmergencyStop ? 'RESUME' : 'E-STOP';
      if (state.isEmergencyStop) {
        audio.playAlarmBuzzer();
        fdd.logEvent('ESTOP-01', 'danger', 'EMERGENCY STOP BUTTON ENGAGED. Pneumatic & conveyor circuits disabled.');
      } else {
        fdd.logEvent('ESTOP-CLR', 'info', 'Emergency stop released. System resumed normal operation.');
      }
      updateOverallSystemStatus();
    });
  }

  // Supply Pressure Slider
  const pressureSlider = document.getElementById('supply-pressure-slider');
  const pressureLabel = document.getElementById('slider-pressure-label');
  if (pressureSlider) {
    pressureSlider.addEventListener('input', (e) => {
      const val = parseFloat(e.target.value);
      state.nominalSupplyPressure = val;
      if (pressureLabel) pressureLabel.textContent = `${val.toFixed(1)} bar`;
    });
  }

  // Feed Speed Slider
  const speedSlider = document.getElementById('feed-speed-slider');
  const speedLabel = document.getElementById('feed-speed-label');
  if (speedSlider) {
    speedSlider.addEventListener('input', (e) => {
      const val = parseFloat(e.target.value);
      state.feedIntervalSec = val;
      if (speedLabel) speedLabel.textContent = `${val.toFixed(1)}s`;
    });
  }

  // Manual Inject Buttons
  const injectMetalBtn = document.getElementById('feed-metal-btn');
  if (injectMetalBtn) {
    injectMetalBtn.addEventListener('click', () => {
      sim.spawnSpecimen('metal');
    });
  }

  const injectPlasticBtn = document.getElementById('feed-plastic-btn');
  if (injectPlasticBtn) {
    injectPlasticBtn.addEventListener('click', () => {
      sim.spawnSpecimen('nonmetal');
    });
  }

  // Fault Injection Suite (Buttons for Viva/Demonstration)
  setupFaultButton('inject-leak-btn', 'airleak', 'AIR LEAKAGE FAULT');
  setupFaultButton('inject-lowpress-btn', 'lowpressure', 'LOW PRESSURE FAULT', () => {
    state.nominalSupplyPressure = 3.2;
    state.actualPressure = 3.2;
    if (pressureSlider) pressureSlider.value = 3.2;
    if (pressureLabel) pressureLabel.textContent = '3.2 bar';
  });
  setupFaultButton('inject-jam-btn', 'cylinderext', 'CYLINDER MECHANICAL JAM');
  setupFaultButton('inject-miss-btn', 'sortingmismatch', 'SORTING VERIFICATION MISMATCH');
  setupFaultButton('inject-sensor-btn', 'sensorfault', 'INDUCTIVE SENSOR FAILURE');

  // Acknowledge All Alarms
  const ackBtn = document.getElementById('ack-alarms-btn');
  if (ackBtn) {
    ackBtn.addEventListener('click', () => {
      for (const k of Object.keys(state.faults)) {
        state.faults[k] = false;
        state.injected[k] = false;
        fdd.updateUIFaultState(k, false);
      }
      fdd.logEvent('ACK-001', 'info', 'Operator acknowledged all diagnostic alerts.');
    });
  }

  // Reset Simulation
  const resetBtn = document.getElementById('reset-sim-btn');
  if (resetBtn) {
    resetBtn.addEventListener('click', () => {
      state.nominalSupplyPressure = 6.2;
      state.actualPressure = 6.2;
      state.pressureDecayRate = 0.012;
      state.cylinderStrokeTimeMs = 118;
      state.cylinderPos = 0.0;
      state.cylinderTarget = 0.0;
      state.solenoidEnergized = false;
      state.pressureHistory = new Array(120).fill(6.2);

      const pressureSlider = document.getElementById('supply-pressure-slider');
      const pressureLabel = document.getElementById('slider-pressure-label');
      if (pressureSlider) pressureSlider.value = 6.2;
      if (pressureLabel) pressureLabel.textContent = '6.2 bar';

      state.counts.metal = 0;
      state.counts.nonmetal = 0;
      state.counts.total = 0;
      state.counts.missed = 0;
      state.cylinderActuations = 0;
      state.specimens = [];

      for (const k of Object.keys(state.faults)) {
        state.faults[k] = false;
        state.injected[k] = false;
        fdd.updateUIFaultState(k, false);
      }
      updateOverallSystemStatus();
      fdd.logEvent('RESET-01', 'info', 'Test bench counters and parameters reset to default 6.2 bar.');
      updateKPICounters();
    });
  }

  // Quick Restore Button in Navbar
  const quickRestoreBtn = document.getElementById('quick-restore-btn');
  if (quickRestoreBtn) {
    quickRestoreBtn.addEventListener('click', () => {
      if (resetBtn) resetBtn.click();
    });
  }

  // Conveyor Speed Calibration Controls
  const calibApplyBtn = document.getElementById('calib-apply-btn');
  if (calibApplyBtn) calibApplyBtn.addEventListener('click', applyCalibration);
  const calibResetBtn = document.getElementById('calib-reset-btn');
  if (calibResetBtn) calibResetBtn.addEventListener('click', resetCalibration);

  // Export Diagnostics CSV
  const exportBtn = document.getElementById('export-csv-btn');
  if (exportBtn) {
    exportBtn.addEventListener('click', exportShiftReportCSV);
  }

  // Feed Mode Select
  const feedSelect = document.getElementById('feed-mode-select');
  const plcBadge = document.getElementById('plc-mode-badge');
  if (feedSelect && plcBadge) {
    feedSelect.addEventListener('change', (e) => {
      state.feedMode = e.target.value;
      if (e.target.value === 'simulated') {
        state.live.connected = false;
        if (plcSocket) plcSocket.close();
        plcBadge.textContent = 'PLC: SIMULATED';
      } else if (e.target.value === 'websocket') {
        plcBadge.textContent = 'PLC: WS DISCONNECTED';
        tryConnectWebSocket();
      } else {
        plcBadge.textContent = 'PLC: MODBUS GATEWAY';
      }
    });
  }
}

function setupFaultButton(btnId, faultKey, label, onActivate) {
  const btn = document.getElementById(btnId);
  if (!btn) return;

  btn.addEventListener('click', () => {
    const willActive = !state.faults[faultKey];
    if (willActive) {
      state.injected[faultKey] = true;
      if (onActivate) onActivate();
      fdd.raiseFault(faultKey, `SIM-${faultKey.toUpperCase()}`, `Manually injected ${label} via test bench.`);
    } else {
      state.injected[faultKey] = false;
      fdd.clearFault(faultKey);
      if (faultKey === 'lowpressure' || faultKey === 'airleak') {
        state.nominalSupplyPressure = 6.2;
        state.actualPressure = 6.2;
        state.pressureDecayRate = 0.012;
        state.cylinderStrokeTimeMs = 118;
        state.cylinderPos = 0.0;
        state.cylinderTarget = 0.0;
        const pressureSlider = document.getElementById('supply-pressure-slider');
        const pressureLabel = document.getElementById('slider-pressure-label');
        if (pressureSlider) pressureSlider.value = 6.2;
        if (pressureLabel) pressureLabel.textContent = '6.2 bar';
      }
      updateOverallSystemStatus();
    }
  });
}

function exportShiftReportCSV() {
  const headers = ['Timestamp', 'Log Code', 'Severity', 'Message'];
  const rows = state.alarmLog.map(e => [
    `"${e.time}"`,
    `"${e.code}"`,
    `"${e.type}"`,
    `"${e.msg.replace(/"/g, '""')}"`
  ]);

  const summary = [
    ['--- SHIFT TELEMETRY SUMMARY ---'],
    [`Supply Pressure: ${state.actualPressure.toFixed(2)} bar`],
    [`Metals Sorted (Bin 1): ${state.counts.metal}`],
    [`Non-Metals Handled (Bin 2): ${state.counts.nonmetal}`],
    [`Total Throughput: ${state.counts.total}`],
    [`Solenoid Valve Actuations: ${state.cylinderActuations}`],
    [`Average Stroke Time: ${state.cylinderStrokeTimeMs} ms`],
    ['--- EVENT LOG ---'],
    headers.join(',')
  ];

  const csvContent = 'data:text/csv;charset=utf-8,' + 
    summary.join('\n') + '\n' + 
    rows.map(r => r.join(',')).join('\n');

  const encodedUri = encodeURI(csvContent);
  const link = document.createElement('a');
  link.setAttribute('href', encodedUri);
  link.setAttribute('download', `PneumoTrack_ShiftReport_${Date.now()}.csv`);
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
}

// ============================================================================
// CONVEYOR SPEED AUTO-CALIBRATION
// ============================================================================
// The canvas animates at BELT_LAYOUT scale; the real rig runs at whatever speed
// its motor runs at. If the two disagree, the animated workpiece reaches the
// pusher at the wrong moment and the twin reports phantom sort mismatches.
// A median over several pieces rejects the odd piece that slips or is hand-fed.

const CALIB_MIN_SAMPLES = 3;
const CALIB_MAX_SAMPLES = 15;

function median(values) {
  if (!values.length) return null;
  const sorted = [...values].sort((a, b) => a - b);
  const mid = Math.floor(sorted.length / 2);
  return sorted.length % 2 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2;
}

function recordCalibrationSample(transitSec) {
  // Reject implausible intervals: a missed edge, a piece removed by hand, or a
  // stuck sensor would otherwise poison the median.
  if (!isFinite(transitSec) || transitSec < 0.2 || transitSec > 30) return;

  const c = state.calibration;
  c.samples.push(transitSec);
  if (c.samples.length > CALIB_MAX_SAMPLES) c.samples.shift();

  const med = median(c.samples);
  c.suggestedSpeed = med ? INFEED_TO_INDUCTIVE_PX / med : null;

  if (c.samples.length === CALIB_MIN_SAMPLES) {
    fdd.logEvent('CAL-001', 'info', `Conveyor calibration has ${CALIB_MIN_SAMPLES} samples. Suggested speed: ${c.suggestedSpeed.toFixed(1)} px/s.`);
  }
}

function resetCalibration() {
  state.calibration.samples = [];
  state.calibration.infeedEdgeAt = 0;
  state.calibration.suggestedSpeed = null;
}

function applyCalibration() {
  const c = state.calibration;
  if (c.suggestedSpeed === null || c.samples.length < CALIB_MIN_SAMPLES) return;

  const previous = state.conveyorSpeed;
  state.conveyorSpeed = parseFloat(c.suggestedSpeed.toFixed(1));
  fdd.logEvent('CAL-002', 'info', `Conveyor speed calibrated from ${previous} to ${state.conveyorSpeed} px/s over ${c.samples.length} measured transits.`);
}

function updateCalibrationUI() {
  const statusEl = document.getElementById('calib-status');
  const samplesEl = document.getElementById('calib-samples');
  const transitEl = document.getElementById('calib-transit');
  const suggestedEl = document.getElementById('calib-suggested');
  const applyBtn = document.getElementById('calib-apply-btn');
  if (!statusEl) return;

  const c = state.calibration;
  const med = median(c.samples);
  const ready = c.samples.length >= CALIB_MIN_SAMPLES && c.suggestedSpeed !== null;

  if (!isLiveTelemetry()) {
    statusEl.textContent = 'Needs a live PLC link - switch Feed Mode to WebSocket.';
  } else if (!c.samples.length) {
    statusEl.textContent = 'Live. Run metal workpieces through to collect samples.';
  } else if (!ready) {
    statusEl.textContent = `Collecting - ${CALIB_MIN_SAMPLES - c.samples.length} more metal piece(s) needed.`;
  } else {
    statusEl.textContent = `Ready. Current speed ${state.conveyorSpeed} px/s.`;
  }

  if (samplesEl) samplesEl.textContent = String(c.samples.length);
  if (transitEl) transitEl.textContent = med ? `${med.toFixed(2)} s` : '--';
  if (suggestedEl) suggestedEl.textContent = ready ? `${c.suggestedSpeed.toFixed(1)} px/s` : '--';
  if (applyBtn) applyBtn.disabled = !ready;
}

// ============================================================================
// LIVE PLC TELEMETRY LINK  (hardware_bridge.js -> ws://localhost:8080)
// ============================================================================

let plcSocket = null;

function setPlcLinkState(connected, badgeText) {
  state.live.connected = connected;
  const badge = document.getElementById('plc-mode-badge');
  if (badge) {
    badge.textContent = badgeText;
    badge.style.color = connected ? '#34d399' : '';
  }
}

/**
 * Applies one telemetry frame from the Modbus bridge to the dashboard state.
 * Frame shape (see hardware_bridge.js):
 *   { timestamp, source, pressure,
 *     sensors:   { inductive, irInfeed, irBin1 },
 *     actuators: { solenoid, conveyor, buzzer } }
 */
function ingestPlcTelemetry(frame) {
  state.live.lastPacket = performance.now();

  if (typeof frame.pressure === 'number' && isFinite(frame.pressure)) {
    state.actualPressure = frame.pressure;
    state.nominalSupplyPressure = Math.max(state.nominalSupplyPressure, frame.pressure);
  }

  if (frame.sensors) {
    const infeed = !!frame.sensors.irInfeed;

    state.sensors.inductive = !!frame.sensors.inductive;
    state.sensors.irInfeed = infeed;
    state.sensors.irBin1 = !!frame.sensors.irBin1;

    // The PLC has no notion of where a workpiece sits on the belt - it only
    // reports sensor bits. So the digital twin derives the belt contents from
    // the infeed IR's rising edge and advances them at the known conveyor
    // speed. A piece starts out assumed non-metallic and is reclassified if the
    // inductive sensor asserts while it is inside the 8mm sensing window.
    if (infeed && !state.live.lastInfeedIR) {
      sim.spawnSpecimen('nonmetal');
      state.calibration.infeedEdgeAt = state.live.lastPacket;
    }
    state.live.lastInfeedIR = infeed;

    // Inductive rising edge closes a calibration interval. Only metal trips it,
    // so non-metal pieces simply never complete a sample.
    const ind = !!frame.sensors.inductive;
    if (ind && !state.live.lastInductive && state.calibration.infeedEdgeAt) {
      recordCalibrationSample((state.live.lastPacket - state.calibration.infeedEdgeAt) / 1000);
      state.calibration.infeedEdgeAt = 0;
    }
    state.live.lastInductive = ind;
  }

  if (frame.actuators) {
    const coil = !!frame.actuators.solenoid;

    // triggerCylinder() is bypassed on a live link (the PLC owns the valve), so
    // the stroke bookkeeping it normally does has to happen on the coil's
    // rising edge instead. Without this lastStrokeStart never advances, the
    // FDD's "line is idle" test is always true, and the pressure dip of a
    // normal actuation gets misread as ERR-01 air leakage.
    if (coil && !state.live.lastSolenoid) {
      state.lastStrokeStart = performance.now();
      state.cylinderActuations++;
      audio.playPneumaticHiss();
    }

    state.live.lastSolenoid = coil;
    state.solenoidEnergized = coil;
    state.conveyorRunning = !!frame.actuators.conveyor;
  }
}

function tryConnectWebSocket() {
  if (plcSocket && (plcSocket.readyState === WebSocket.OPEN || plcSocket.readyState === WebSocket.CONNECTING)) {
    return;
  }

  try {
    plcSocket = new WebSocket('ws://localhost:8080');
  } catch {
    setPlcLinkState(false, 'PLC: WS OFFLINE (SIM ACTIVE)');
    return;
  }

  plcSocket.onopen = () => {
    setPlcLinkState(true, 'PLC: WS CONNECTED');
    state.live.lastPacket = performance.now();
    resetCalibration();
    fdd.logEvent('WS-001', 'info', 'Connected to PLC bridge. Internal simulation standing down - telemetry is now live.');
  };

  plcSocket.onmessage = (ev) => {
    try {
      const frame = JSON.parse(ev.data);
      if (!state.live.connected) {
        setPlcLinkState(true, 'PLC: WS CONNECTED');
      }
      ingestPlcTelemetry(frame);
    } catch (err) {
      console.warn('[SCADA] Malformed telemetry frame from PLC bridge:', err);
    }
  };

  plcSocket.onerror = () => {
    setPlcLinkState(false, 'PLC: WS OFFLINE (SIM ACTIVE)');
  };

  plcSocket.onclose = () => {
    setPlcLinkState(false, 'PLC: WS OFFLINE (SIM ACTIVE)');
    // Keep retrying only while the operator has the link selected.
    if (state.feedMode === 'websocket') {
      setTimeout(tryConnectWebSocket, 3000);
    }
  };
}

// Live Clock in Navbar
function updateClock() {
  const clockEl = document.getElementById('live-clock');
  if (clockEl) {
    clockEl.textContent = new Date().toLocaleTimeString();
  }
}
setInterval(updateClock, 1000);
updateClock();

// ============================================================================
// MAIN ANIMATION & RENDER LOOP
// ============================================================================
function animationLoop(timestamp) {
  // 1. Advance Physics & Logic Simulation
  sim.step(timestamp);

  // 2. Render Digital Twin Visual Canvas
  renderDigitalTwin();

  // 3. Render Pressure Waveform Canvas
  renderPressureWaveform();

  // 4. Update Gauges, Meters, and Badges
  updateLiveGaugesAndBadges();

  requestAnimationFrame(animationLoop);
}

// Initialize Application
document.addEventListener('DOMContentLoaded', () => {
  setupEventListeners();
  updateKPICounters();
  renderAlarmLog();
  requestAnimationFrame(animationLoop);
});
