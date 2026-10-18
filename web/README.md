# FuelGuard+ Lorry — 3D viewer

A single self-contained page: a cab-over tipper lorry that drives back and
forth, with a transparent saddle fuel tank (visible fuel level) and an open
control kit showing the actual breadboard build — two breadboards, ESP32,
MPU6050, buzzer, GPS module + antenna, switch, and boost converter, each
labeled.

## Run it

No build step, no npm install, no local server needed — Three.js loads
straight from a CDN via plain `<script>` tags (not ES modules), so you can
just open the file directly in a browser:

1. Open this folder in VS Code.
2. Right-click `index.html` → **Open with Live Server** (if you have the
   extension), *or* just double-click `index.html` in your file explorer to
   open it straight in a browser — either works.

## Controls

- Drag to orbit, scroll to zoom.
- The lorry drives itself back and forth — no interaction needed.

## Editing

Everything lives in `index.html` in one `<script>` block: materials near the
top (`const M = {...}`), the lorry geometry below that, and the drive
animation at the bottom (`driveSpan` controls how far it travels).

## v2 UI update

- **Right side:** FuelGuard kit assembly (tank with fuel/water layers, float + water sensors, vibration sensor, ESP32, MPU6050, GPS, GSM, voltage sensor) with animated wiring, plus a compact **Components Used** card grid.
- **GPS is interactive:** click the GPS card, the GPS module in the kit drawing, or the "GPS tracking" header chip. The page scrolls smoothly to the GPS section, highlights GPS everywhere (card, kit drawing, 3D roof antenna pulses), and reveals the explanation.
- **GPS section:** live map marker follows the lorry's driving animation, with coordinates and speed readouts. "Simulate theft alert" shows the alert SMS with the vehicle location.
