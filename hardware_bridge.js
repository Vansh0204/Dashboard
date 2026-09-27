/**
 * PNEUMOTRACK PLC HARDWARE INTEGRATION BRIDGE
 * ===========================================
 * Connects your Physical PLC (Siemens / Delta / Mitsubishi / Schneider) 
 * to the PneumoTrack SCADA Web Dashboard via Modbus TCP & WebSockets.
 * 
 * Requirements:
 * npm install modbus-serial ws
 * 
 * Run:
 * node hardware_bridge.js
 */

import { WebSocketServer } from 'ws';
import ModbusRTU from 'modbus-serial';

// CONFIGURATION: Set your PLC's IP address and Port
const PLC_CONFIG = {
  ip: '192.168.0.1', // Default PLC Ethernet IP
  port: 502,         // Standard Modbus TCP Port
  unitId: 1          // Modbus Slave ID
};

const WS_PORT = 8080;

// Initialize Modbus Client
const modbusClient = new ModbusRTU();
let isPlcConnected = false;

async function connectToPLC() {
  try {
    console.log(`[PLC BRIDGE] Attempting connection to PLC at ${PLC_CONFIG.ip}:${PLC_CONFIG.port}...`);
    await modbusClient.connectTCP(PLC_CONFIG.ip, { port: PLC_CONFIG.port });
    modbusClient.setID(PLC_CONFIG.unitId);
    modbusClient.setTimeout(1000);
    isPlcConnected = true;
    console.log('[PLC BRIDGE] ✅ Connected to Physical Industrial PLC successfully!');
  } catch (err) {
    isPlcConnected = false;
    console.warn(`[PLC BRIDGE] ⚠️ PLC connection offline (${err.message}). Retrying in 5 seconds...`);
    setTimeout(connectToPLC, 5000);
  }
}

// Start WebSocket server for SCADA Dashboard
const wss = new WebSocketServer({ port: WS_PORT });
console.log(`[PLC BRIDGE] WebSocket Server running on ws://localhost:${WS_PORT}`);

wss.on('connection', (ws) => {
  console.log('[PLC BRIDGE] SCADA Web Dashboard connected via WebSocket.');

  ws.on('message', async (message) => {
    try {
      const command = JSON.parse(message);
      // Example: Dashboard sends E-STOP or Solenoid manual override command
      if (command.type === 'WRITE_COIL' && isPlcConnected) {
        await modbusClient.writeCoil(command.address, command.value);
        console.log(`[PLC BRIDGE] Wrote Coil ${command.address} = ${command.value}`);
      }
    } catch (e) {
      console.error('[PLC BRIDGE] Error processing incoming command:', e);
    }
  });

  ws.on('close', () => {
    console.log('[PLC BRIDGE] SCADA Web Dashboard disconnected.');
  });
});

// Periodic PLC Polling Loop (Every 100ms = 10Hz industrial telemetry)
// Scheduled recursively rather than with setInterval: a Modbus read can take up
// to the 1000ms timeout, and setInterval would stack ten overlapping
// transactions on the same serial client behind it.
const POLL_INTERVAL_MS = 100;

async function pollOnce() {
  if (wss.clients.size === 0) return; // No dashboard connected, skip poll

  let telemetryPayload = {};

  if (isPlcConnected) {
    try {
      // 1. Read Analog Pressure Sensor: Holding Register 100 (%MW100)
      // Scaled in PLC: e.g. 0-1000 = 0.0 - 10.0 bar
      const pressureData = await modbusClient.readHoldingRegisters(100, 1);
      const rawPressure = pressureData.data[0];
      const pressureBar = rawPressure / 100.0;

      // 2. Read Discrete Inputs: %IX0.0 (Inductive), %IX0.1 (Infeed IR), %IX0.2 (Bin 1 IR)
      const inputsData = await modbusClient.readDiscreteInputs(0, 8);
      const inductiveSensor = inputsData.data[0];
      const infeedIR = inputsData.data[1];
      const bin1IR = inputsData.data[2];

      // 3. Read Coils (Outputs): %QX0.0 (Solenoid), %QX0.1 (Conveyor), %QX0.2 (Buzzer)
      const coilsData = await modbusClient.readCoils(0, 8);
      const solenoidState = coilsData.data[0];
      const conveyorState = coilsData.data[1];
      const buzzerState = coilsData.data[2];

      telemetryPayload = {
        timestamp: Date.now(),
        source: 'PHYSICAL_PLC',
        pressure: pressureBar,
        sensors: {
          inductive: inductiveSensor,
          irInfeed: infeedIR,
          irBin1: bin1IR
        },
        actuators: {
          solenoid: solenoidState,
          conveyor: conveyorState,
          buzzer: buzzerState
        }
      };
    } catch (pollErr) {
      console.error('[PLC BRIDGE] Polling error:', pollErr.message);
      isPlcConnected = false;
      modbusClient.close(() => {});
      setTimeout(connectToPLC, 5000);
      return;
    }
  } else {
    // Fallback: If hardware PLC is offline, the dashboard continues on internal simulation
    return;
  }

  // Broadcast live hardware telemetry to all connected SCADA web clients
  const messageStr = JSON.stringify(telemetryPayload);
  wss.clients.forEach((client) => {
    if (client.readyState === 1) {
      client.send(messageStr);
    }
  });
}

async function pollLoop() {
  try {
    await pollOnce();
  } catch (err) {
    console.error('[PLC BRIDGE] Unexpected poll failure:', err.message);
  }
  setTimeout(pollLoop, POLL_INTERVAL_MS);
}

// Start initial connection attempt
connectToPLC();
pollLoop();
