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
| Target-reached LED1 (green) | ESP32 GPIO 12 through 220 Ω to GND | On only when serial closed-loop pressure is within the configured deadband. |
| Active LED2 | ESP32 GPIO 13 through 220 Ω to GND | On when firmware commands nonzero pump PWM or energizes the valve. |
| Mode switch: manual full | ESP32 GPIO 26 with 1 kΩ pulldown | 3.3 V selects pump PWM `255` and valve open. |
| Mode switch: serial control | ESP32 GPIO 27 with 1 kΩ pulldown | 3.3 V enables serial closed-loop control. |
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
| `ZERO` | In serial mode, vent, establish atmospheric offset, and permit negative targets. Required after every reboot. |
| `STATUS` | Print controller state, mode, voltage, pressure, target, PWM, valve state, LED states, ADC address, calibration state, and tuning. |
| `STATUS_STREAM` | Toggle continuous `STATUS` output at 20 Hz. Send `STATUS_STREAM` again to stop it. |
| `KP <value>` | Set proportional gain in RAM; allowed range 0–50. |
| `KI <value>` | Set integral gain in RAM; allowed range 0–20. |
| `KD <value>` | Set derivative damping gain in RAM; allowed range 0–10. |
| `BAND <kPa>` | Set deadband in RAM; allowed range 0.1–5 kPa. |
| `SAVE` | Persist current validated tuning in ESP32 Preferences. |
| `DEFAULTS` | Restore conservative default tuning in RAM; send `SAVE` to retain it after reboot. |

Malformed commands stop active control and vent as a precaution.

## Controller behavior

The loop runs every 50 ms and averages four ADS1115 readings per update. PID
control uses anti-windup, a filtered pressure-rate term, and a PWM slew limit.

- If pressure is not negative enough, the valve opens to the pump and PWM rises
  gradually.
- At or beyond the target deadband, the pump turns off while the valve remains
  on the pump-to-gripper path. The controller does not use the full-flow
  atmosphere vent as a corrective actuator.
- Passive venting (valve off) remains the safety/release path for `0`, faults,
  malformed commands, and `ZERO`.

This checkout's defaults are `KP=18`, `KI=1`, `KD=0.3`, and `BAND=0.5 kPa`.
Tune them on the real pump, tubing volume, valve flow, and gripper before
relying on them. In particular, confirm that valve-on/pump-off holds vacuum
without overheating a non-continuous-duty valve coil.

### Switch modes and LEDs

The SPDT mode switch is the output authority. The firmware reads both switch
inputs on every loop and accepts only these complementary states:

| GPIO 26 | GPIO 27 | Mode | Output behavior |
| --- | --- | --- | --- |
| HIGH | LOW | Manual full-drive | Pump PWM is `255` and the valve is energized, even if the ADS1115 is unavailable or faulted. Entering this mode cancels any serial target or calibration in progress. LED2 is on; LED1 is off. |
| LOW | HIGH | Serial closed-loop | The controller remains vented/idle until a new negative serial target is sent. Existing ADC, `ZERO`, target-range, and fault checks apply. |
| LOW | LOW | Safe off | Pump and valve are de-energized. |
| HIGH | HIGH | Safe off | Pump and valve are de-energized. |

Returning from manual full-drive to serial mode does not restore an earlier
target; send `ZERO` if needed, then a new negative target. In manual or
safe-off mode, `ZERO` and numeric target commands are rejected. `STATUS`,
tuning commands, `SAVE`, and `DEFAULTS` remain available.

LED1 is off while the controller ramps, vents, settles, calibrates, idles, is
faulted, or is in manual/safe-off mode. LED2 represents **commanded drive**:
it cannot prove that a disconnected, stalled, or failed motor/valve is drawing
current because this board has no current-sense circuit.

### ADS1115 ADDR override

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


## Idle, off, and fault behavior

The firmware commands **both motor and valve off electrically** when it
is idle or at atmospheric pressure:

```cpp
setOutputs(PWM_OFF, false);
// pump:  analogWrite(GPIO 2, 0)
// valve: analogWrite(GPIO 5, 0)
```

This output path is used at boot, after `0`, after a positive target clamps to
zero, after malformed input, during zeroing, on pressure/ADC faults, and for
invalid mode-switch states. The D26 manual-full mode is the explicit exception:
it deliberately drives both outputs even with a sensor fault. With the stated
plumbing, valve-off is also the pneumatic vent state; it does not attempt to
hold vacuum. Therefore the valve coil is not intentionally kept energized while
idle, avoiding prolonged activation and overheating.

This conclusion assumes a non-inverting valve driver where GPIO 5 HIGH
energizes the coil, as defined by the original controller. Confirm that once
with a multimeter or indicator LED before connecting a gripper.

## First physical test

1. Test with the gripper disconnected or a conservative vented fixture.
2. Select serial mode (GPIO 27 HIGH) and send `STATUS`; verify `MODE=SERIAL`,
   `PWM=0`, `VALVE=VENT`, `LED1=OFF`, `LED2=OFF`, and `ZERO=REQUIRED`.
3. Run `ZERO` with the port open to atmosphere. If possible, compare reported
   voltage with a multimeter at ADS1115 A0.
4. Start with `-5`, then `-10`; verify motor PWM ramps rather than jumps,
   LED2 is on while the pump or valve is commanded, and LED1 turns on only
   after pressure reaches the configured deadband.
5. Send `0` and verify both PWM outputs and both LEDs are off before longer tests.
6. Select manual full-drive (GPIO 26 HIGH) and verify `PWM=255`, `VALVE=OPEN`,
   LED2 on, and LED1 off. Return to serial mode and verify it remains vented
   until a new negative target is sent.
7. Verify both-low and both-high switch input states keep both outputs and LEDs off.
8. Tune one variable at a time and send `SAVE` only after physical validation.

Never treat the −90 kPa software clamp as proof that the gripper, tubing,
fittings, or pump are safe at that pressure.
