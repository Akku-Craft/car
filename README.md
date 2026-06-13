# Elegoo Smart Robot Car V4.0 – IR Remote Control + OLED Speed Display

Controls the car via the included IR remote and displays the current speed state on a 1.3" SH1106 OLED over I2C.

## IR Remote Mapping

| Button | Action |
|--------|--------|
| ▲ UP | Move forward |
| ▼ DOWN | Move backward |
| Any other key (OK, 1-9, etc.) | Stop |
| Hold any button | Repeats ignored, maintains current state |
| No signal for 2s | Safety auto-stop |

Both remote variants (Type A and Type B) are supported.

## Pin Mapping

| Arduino Pin | Function | Connected To |
|-------------|----------|-------------|
| 5 | PWMA | Right motor speed (PWM) |
| 6 | PWMB | Left motor speed (PWM) |
| 7 | AIN1 | Right motor direction |
| 8 | BIN1 | Left motor direction |
| 3 | STBY | Motor standby (HIGH = enabled) |
| 9 | IR input | IR receiver sensor |
| A4 (SDA) | I2C data | OLED SDA |
| A5 (SCL) | I2C clock | OLED SCL |

## Wiring

- **Motor driver**: TB6612 (or DRV8835 — same pinout for this sketch)
- **OLED**: 1.3" SH1106 128×64, I2C interface
- **IR receiver**: Connected to pin 9, power (5V) and GND

## Display

The OLED shows:

```
    FORWARD / REVERSE / STOPPED
    ┌──────────────────────────────┐
    │███████████████               │  ← speed bar
    │ Speed: 200                   │
    └──────────────────────────────┘
```

The display updates instantly on every state change.

## Project Structure

```
car/
├── platformio.ini              # Uno, U8g2 dependency
├── src/
│   └── main.cpp                # Main program
├── lib/
│   ├── IRremote/               # Elegoo's IRremote library
│   │   ├── IRremote.h
│   │   ├── IRremoteInt.h
│   │   └── IRremote.cpp
│   └── README
└── ELEGOO Smart Robot Car Kit V4.0 2023.02.01/
    └── ... (official Elegoo reference code & docs)
```

## Build & Upload

```bash
pio run -t upload
```

Requires [PlatformIO](https://platformio.org/).

## Resource Usage

| | Used | Total |
|---|---|---|
| Flash | 17.8 KB (55%) | 32 KB |
| RAM | 1.8 KB (89%) | 2 KB |
