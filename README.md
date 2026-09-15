# softrobot-electropneu-ctrlsys
Control elements for pneumatic actuated soft robots. Including microcontroller controller firmware, controller PCBs, control board use guidelines and more.

## Closed-loop vacuum controller

This PlatformIO firmware regulates a negative-pressure pneumatic circuit with
an ESP32 DevKit V1, a vacuum-pump motor, a solenoid valve, an ADS1115 ADC, and
a 5 V XGZP6847A pressure sensor. It is set up for the installed
`XGZP6847A100KPGPN` variant: its nominal 0.5–4.5 V output represents −100 to
+100 kPa gauge pressure. The controller only commands negative pressure.

### Hardware and signal path

| Function | Connection | Firmware behavior |
| --- | --- | --- |
| Pump motor PWM | ESP32 GPIO 2 | `0` is off; `255` is full duty cycle. |
| Solenoid valve | ESP32 GPIO 5 | HIGH opens the pump-to-gripper path; LOW de-energizes the valve and vents the gripper. |
| ADS1115 SDA | ESP32 GPIO 21 through the 3.3 V/5 V I2C level shifter | Initialized with `Wire.begin(21, 22)`. |
| ADS1115 SCL | ESP32 GPIO 22 through the 3.3 V/5 V I2C level shifter | Initialized with `Wire.begin(21, 22)`. |
| XGZP6847A output | ADS1115 A0 | Read as a 5 V-domain analog signal; it never connects directly to the ESP32. |

The ADC uses its ±6.144 V range (`GAIN_TWOTHIRDS`), so the sensor's 0.5–4.5 V
signal cannot clip. After field zeroing, the nominal conversion is:

```text
pressure_kPa = 50 × (sensor_volts − zero_volts)
```

`ZERO` removes the atmospheric offset; it does not replace a calibrated span
check against a reference gauge.

### ADS1115 ADDR: temporary test override

For reliable normal operation, wire ADS1115 `ADDR` to GND and use I2C address
`0x48`. The ADS1115 samples this address-selection pin continuously, so a
floating pin is not dependable.

For initial bench testing, `src/main.cpp` includes:

```cpp
const bool ALLOW_FLOATING_ADS1115_ADDR_FOR_TESTING = true;
```

With this temporary toggle enabled, firmware first tries `0x48`, then the
other legal ADS1115 addresses (`0x49`, `0x4A`, `0x4B`). It prints a warning if
the ADC is found away from `0x48`; `STATUS` also reports `ADS_ADDR`. This only
accommodates an accidentally stable floating pin. After wiring `ADDR → GND`,
set this toggle to `false` and rebuild.

## Installation and upload

1. Install [PlatformIO for VS Code](https://platformio.org/install/ide?install=vscode)
   and open this repository as a PlatformIO project.
2. Confirm `platformio.ini` targets `esp32doit-devkit-v1`.
3. Build. PlatformIO installs `Adafruit ADS1X15` automatically; `Arduino`,
   `Wire`, and ESP32 `Preferences` are framework libraries.

   ```sh
   pio run
   ```

4. Connect the ESP32 by USB and upload:

   ```sh
   pio run --target upload
   ```

5. Open a serial monitor at **115200 baud** with line ending set to `Newline`:

   ```sh
   pio device monitor --baud 115200
   ```

Boot starts deliberately uncalibrated. The firmware leaves the pump and valve
off and rejects negative targets until `ZERO` succeeds.

## Operation

With the pressure port open to atmosphere, first send:

```text
ZERO
```

The controller turns both outputs off for one second, then averages 32 stable
pressure samples. Do not obstruct the vent or run the pump while it calibrates.

After `ZERO complete`, send a negative gauge-pressure target as a bare number:

```text
-20
-65.93
```

The accepted range is −90 to 0 kPa. Values below −90 kPa clamp to −90; positive
values clamp to zero and vent. Any zero representation stops control and vents:

```text
0
0.0
-0.0
```

### Serial commands

| Command | Effect |
| --- | --- |
| `ZERO` | Vent, establish atmospheric offset, and permit negative targets. Required after every reboot. |
| `STATUS` | Print controller state, voltage, pressure, target, PWM, valve state, ADC address, calibration state, and tuning. |
| `KP <value>` | Set proportional gain in RAM; allowed range 0–50. |
| `KI <value>` | Set integral gain in RAM; allowed range 0–20. |
| `KD <value>` | Set derivative damping gain in RAM; allowed range 0–10. |
| `BAND <kPa>` | Set deadband in RAM; allowed range 0.1–5 kPa. |
| `VENT <ms>` | Set overshoot vent-pulse duration in RAM; allowed range 5–100 ms. |
| `SAVE` | Persist current validated tuning in ESP32 Preferences. |
| `DEFAULTS` | Restore conservative default tuning in RAM; send `SAVE` to retain it after reboot. |

Malformed commands stop active control and vent as a precaution.

## Controller behavior

The loop runs every 50 ms and averages four ADS1115 readings per update. PID
control uses anti-windup, a filtered pressure-rate term, and a PWM slew limit.

- If pressure is not negative enough, the valve opens to the pump and PWM rises
  gradually.
- If pressure becomes too negative, the pump turns off and the valve receives a
  short LOW vent pulse; the controller waits 200 ms before correcting again.
- Inside the deadband, the valve remains on the pump path and PWM is adjusted
  only as needed to counter leakage.

Initial tuning is deliberately conservative: `KP=12`, `KI=1`, `KD=0.3`,
`BAND=0.5 kPa`, and `VENT=20 ms`. Tune it on the real pump, tubing volume,
valve flow, and gripper before relying on it.

## Idle, off, and fault behavior

Yes: the firmware commands **both motor and valve off electrically** when it
is idle or at atmospheric pressure:

```cpp
setOutputs(PWM_OFF, false);
// pump:  analogWrite(GPIO 2, 0)
// valve: analogWrite(GPIO 5, 0)
```

This output path is used at boot, after `0`, after a positive target clamps to
zero, after malformed input, during zeroing, and on pressure/ADC faults. With
the stated plumbing, valve-off is also the pneumatic vent state; it does not
attempt to hold vacuum. Therefore the valve coil is not intentionally kept
energized while idle, avoiding prolonged activation and overheating.

This conclusion assumes a non-inverting valve driver where GPIO 5 HIGH
energizes the coil, as defined by the original controller. Confirm that once
with a multimeter or indicator LED before connecting a gripper.

## First physical test

1. Test with the gripper disconnected or a conservative vented fixture.
2. Boot and send `STATUS`; verify `PWM=0`, `VALVE=VENT`, and `ZERO=REQUIRED`.
3. Run `ZERO` with the port open to atmosphere. If possible, compare reported
   voltage with a multimeter at ADS1115 A0.
4. Start with `-5`, then `-10`; verify motor PWM ramps rather than jumps.
5. Send `0` and verify both PWM outputs are zero before longer tests.
6. Tune one variable at a time and send `SAVE` only after physical validation.

Never treat the −90 kPa software clamp as proof that the gripper, tubing,
fittings, or pump are safe at that pressure.
