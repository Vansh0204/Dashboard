# PneumoTrack SCADA 4.0 — Project Guide

**Real-Time Pneumatic System Monitoring and Fault Detection Dashboard**

Department of Mechatronics Engineering, School of Automation, Banasthali Vidyapith
Group 44 — Khushi Agarwal, Astha Singh, Mishti Khandelwal, Akansha Pandey, Himani Sahu
Supervisor: Dr. Vineet Pandey

This guide explains what the project is and how to connect it to the physical machine.
It assumes no programming background. Technical terms are explained the first time
they appear.

---

## 1. What this project is, in one paragraph

We built a machine that sorts objects into two bins — metal in one, non-metal in the
other. This project is the **screen that watches that machine**. It shows the air
pressure, the sensors switching on and off, the pusher firing, how many pieces have
gone into each bin, and it raises an alarm when something goes wrong. Think of it as
the dashboard of a car: the car still runs without it, but the dashboard is how you
know the engine is healthy.

In industry this kind of screen is called **SCADA** — Supervisory Control and Data
Acquisition. It is the standard way a factory operator watches a machine from a desk
instead of standing next to it.

---

## 2. The physical machine

Objects travel along a moving belt. Along the way:

1. An **infeed sensor** notices an object has entered the belt.
2. A **proximity sensor** checks whether the object is metal. This sensor only
   reacts to metal — plastic and wood pass by without triggering it.
3. If the object is metal, a **pneumatic cylinder** (a push rod driven by compressed
   air) shoves it sideways into **Bin 1**.
4. If it is not metal, nothing happens and it continues to the end of the belt and
   falls into **Bin 2**.

Compressed air does the pushing. A **solenoid valve** is the electrically operated
tap that lets air into the cylinder at the right moment.

All of this is controlled by a **PLC** (Programmable Logic Controller) — a rugged
industrial computer built for exactly this job. The PLC reads the sensors and decides
when to fire the valve.

> **Why a PLC and not an ESP32?** The original plan used an ESP32 microcontroller,
> which works at 3.3 volts. The industrial sensors and valves on this rig need 12–24
> volts. Using a PLC removes the need for voltage conversion circuitry between every
> component. This was a deliberate design change, and it is worth mentioning in the
> viva.

---

## 3. What the dashboard shows

| Section | What it tells you |
|---|---|
| Supply Pressure | Current air pressure in the line, in bar |
| Leakage Rate | How fast pressure is dropping — a fast drop means an air leak |
| Metals / Non-Metals | Running count of pieces in each bin, and the sorting accuracy |
| Cylinder Stroke Time | How long the push rod takes to extend, in milliseconds |
| Digital Twin | A live animated drawing of the machine, updating as it runs |
| Pressure Waveform | A rolling graph of pressure over time |
| Fault Diagnosis | Five specific failures, each shown as OK or FAULT |
| PLC Register Map | The raw signal values the PLC is reporting |
| Alarm Log | A timestamped history of everything that happened |

### The five faults it detects

| Code | Fault | How it is detected |
|---|---|---|
| ERR-01 | Air line leakage | Pressure falling faster than 0.05 bar per second |
| ERR-02 | Low supply pressure | Line pressure below 4.5 bar |
| ERR-03 | Cylinder jam | Valve fired but the rod did not fully extend within 250 ms |
| ERR-04 | Sorting failure | A metal piece ended up in the wrong bin |
| ERR-05 | Sensor abnormality | The proximity sensor is stuck in one state |

There is also a **Fault Injection** panel with buttons that deliberately trigger each
fault. This is for demonstration — you can show an examiner exactly how the system
reacts to a leak without actually damaging anything.

---

## 4. It works right now, with no hardware attached

This matters: **the dashboard runs perfectly well on its own.** It contains a built-in
simulation of the machine — imaginary objects travel along the belt, pressure rises
and falls realistically, and the pusher fires. Everything on screen is live and moving.

So you can demonstrate the entire project on a laptop, with no rig, no PLC and no air
compressor. That is the safe fallback if hardware is unavailable on the day.

To run it:

```bash
npm install
npm run dev
```

Then open **http://localhost:5173/** in a browser.

Connecting the real machine is an upgrade on top of this, not a requirement.

---

## 5. How the hardware connection works

### Why a "bridge" program is needed

There is one obstacle. The PLC speaks an industrial language called **Modbus TCP**.
A web browser cannot speak Modbus — browsers were never designed to talk to factory
equipment directly. It is a genuine technical limitation, not an oversight.

So there is a small translator program in the middle, called the **bridge**
(`hardware_bridge.js`). It speaks Modbus to the PLC, and speaks the browser's language
back to the dashboard. It asks the PLC for fresh readings **ten times every second**.

The full chain:

```
   PLC   ───── Modbus TCP ─────►   bridge program   ───── WebSocket ─────►   dashboard
(the machine)   ethernet cable     (runs on laptop)       (same laptop)        (browser)
```

Three things must be running for live data: the PLC, the bridge, and the dashboard.

---

## 6. Connecting to the hardware — step by step

### Step 1 — Prepare the PLC program

*This step happens in your PLC software, not in this project.*

The PLC must publish its readings at specific numbered addresses, so the bridge knows
where to look. Think of these as numbered pigeonholes that both sides have agreed on
in advance.

| Signal | Address | Notes |
|---|---|---|
| Air pressure | Holding register **100** | Must be scaled so 0–1000 means 0.00–10.00 bar |
| Proximity sensor (metal) | Discrete input **0** | |
| Infeed sensor | Discrete input **1** | |
| Bin 1 sensor | Discrete input **2** | |
| Solenoid valve | Coil **0** | |
| Conveyor motor | Coil **1** | |
| Buzzer | Coil **2** | |

The pressure scaling is the one to double-check. The bridge divides that number by 100
to get bar. If your PLC scales pressure differently, that division must be adjusted in
`hardware_bridge.js` — otherwise the pressure reading will be wrong by a factor of ten
and every alarm will misfire.

### Step 2 — Connect the network

Connect the laptop and the PLC to the same network with an ethernet cable, and make
sure both have addresses on the same subnet. If the PLC is at `192.168.0.1`, the
laptop should be something like `192.168.0.50`.

To check the PLC is reachable, open a terminal and run:

```bash
ping 192.168.0.1
```

If replies come back, the network is fine. If it times out, fix that before going
further — nothing downstream will work.

### Step 3 — Tell the bridge where the PLC is

Open `hardware_bridge.js` in any text editor. The first few lines are:

```js
const PLC_CONFIG = {
  ip: '192.168.0.1', // Default PLC Ethernet IP
  port: 502,         // Standard Modbus TCP Port
  unitId: 1          // Modbus Slave ID
};
```

Change the IP address to your PLC's actual address. Leave the port at 502 unless your
PLC documentation says otherwise. Save the file.

### Step 4 — Start the bridge

Open a terminal in the project folder and run:

```bash
npm run bridge
```

Leave this terminal open. You should see:

```
[PLC BRIDGE] WebSocket Server running on ws://localhost:8080
[PLC BRIDGE] Connected to Physical Industrial PLC successfully!
```

If instead it repeats `PLC connection offline ... Retrying in 5 seconds`, the PLC is
not reachable. Go back to Step 2.

### Step 5 — Start the dashboard

Open a **second** terminal — do not close the first one — and run:

```bash
npm run dev
```

Open **http://localhost:5173/** in a browser.

### Step 6 — Switch the dashboard to live mode

On the dashboard, find **Feed Mode** in the PLC panel and change it from
`Internal PLC Simulator` to `Live WebSocket`.

The badge at the top should change to **PLC: WS CONNECTED**, and the simulation will
stop. From this point the numbers on screen are coming from the real machine.

If the machine stops sending data for 3 seconds, the dashboard automatically switches
back to simulation and says so. It will not freeze or show stale readings.

### Step 7 — Calibrate the belt speed

One number needs tuning the first time. The dashboard's animated belt needs to run at
the same speed as the real one, otherwise the drawing drifts out of step with the
machine.

The dashboard measures this for you:

1. In the PLC panel, find the **Conveyor Speed Calibration** strip.
2. Run at least **three metal pieces** through the machine. Metal specifically —
   the measurement depends on the proximity sensor firing, and non-metal pieces do
   not trigger it.
3. The suggested speed appears. Press **Apply**.

To make it permanent, copy that number into `conveyorSpeed` in `src/main.js`. If you
skip this, the value resets the next time the page is reloaded.

---

## 7. What is genuinely live, and what is not

Be clear about this — an examiner may well ask, and the honest answer is a strong one.

**Genuinely measured from the machine:**

- Air pressure
- The leakage rate, calculated from real pressure readings
- All three sensors switching on and off
- The valve, conveyor motor and buzzer outputs
- The cylinder extending and retracting
- All five fault detections, running on the above

**Drawn, not measured — the objects moving along the belt:**

A PLC reports sensor signals, not object positions. It knows "the infeed sensor just
switched on"; it does not know where any object is sitting on the belt. No PLC does.

So the dashboard works it out: when the infeed sensor triggers, it starts drawing an
object, moves it along at the calibrated speed, and marks it as metal if the proximity
sensor fires as it passes. The positions are inferred from real sensor events.

This is normal for industrial SCADA systems and is not a weakness in the design. It is
simply why Step 7 exists — get the speed wrong, and the drawing and the machine will
disagree.

---

## 8. If something does not work

| What you see | What it means | What to do |
|---|---|---|
| Bridge repeats "connection offline" | Laptop cannot reach the PLC | Check the cable and IP addresses; try `ping` |
| Badge stays "WS OFFLINE" | The bridge is not running | Check the first terminal is still open |
| Badge flips to "WS TIMEOUT" | Bridge is running but the PLC has gone quiet | Check PLC power and program is in RUN |
| Pressure reads ten times too high or low | Scaling mismatch | Check the divide-by-100 in `hardware_bridge.js` |
| Animated object arrives at the wrong moment | Belt speed not calibrated | Redo Step 7 |
| Pieces counted in the wrong bin | Usually belt speed, occasionally a sensor | Redo Step 7 first |

---

## 9. One safety point

The **E-STOP button on the dashboard stops the display only. It does not stop the
machine.** Control currently runs one way: the PLC sends information to the dashboard,
and the dashboard does not send commands back.

Always use the physical emergency stop on the rig as the real safety device. Never rely
on a button in a web browser to stop moving machinery — a browser can freeze, the
network can drop, and neither failure would be obvious in the moment.

---

## 10. Quick reference

| Task | Command |
|---|---|
| Install everything (first time only) | `npm install` |
| Run the dashboard | `npm run dev` |
| Run the hardware bridge | `npm run bridge` |
| Dashboard address | http://localhost:5173/ |

**Demonstrating without hardware:** run `npm run dev` only, leave Feed Mode on
`Internal PLC Simulator`, and use the Fault Injection buttons to show each alarm.

**Demonstrating with hardware:** run the bridge as well, switch Feed Mode to
`Live WebSocket`, and calibrate once before starting.
