# Automatic Distance Alert System

An Arduino Uno based proximity alert unit. An HC-SR04 ultrasonic sensor measures
the distance to the nearest object in front of it. When that object comes inside
a configurable threshold, an LED lights and a buzzer sounds; outside the
threshold both switch off. Live distance readings are streamed to the serial
monitor.

**Simulation:** <paste your Wokwi link here>

---

## Requirements covered

| Requirement | How it is met |
|---|---|
| Detect objects in front | HC-SR04 ultrasonic sensor, 2–400 cm |
| Trigger LED / buzzer inside threshold | LED on D12, piezo buzzer on D11 |
| Turn off outside threshold | Hysteresis band prevents boundary flicker |
| Configurable threshold | Set live over serial (`T30`), saved to EEPROM |
| Display distance via serial monitor | Printed every 250 ms with status |

## Hardware

| Component | Arduino pin |
|---|---|
| HC-SR04 VCC | 5V |
| HC-SR04 GND | GND |
| HC-SR04 TRIG | D9 |
| HC-SR04 ECHO | D10 |
| Buzzer (+) | D11 |
| LED anode (via 220 Ω) | D12 |
| LED cathode / buzzer (−) | GND |

## How it works

1. **Measurement.** A 10 µs pulse on TRIG emits an ultrasonic burst. The width
   of the returning ECHO pulse is the round-trip time; distance =
   `echo_time × 0.0343 / 2` cm. `pulseIn()` uses a 25 ms timeout so a missing
   echo never stalls the program.
2. **Filtering.** Each measurement is the **median of three pings**, so a single
   corrupt reading cannot produce a false alert.
3. **Decision.** The alert trips at `distance ≤ threshold` and only releases at
   `distance > threshold + 2 cm`. This **hysteresis** keeps the output stable
   when an object hovers on the boundary.
4. **Feedback.** The LED is steady while the alert is active; the buzzer beep
   interval scales from 450 ms at the threshold down to 70 ms at point-blank
   range, giving an audible sense of how close the object is.
5. **Timing.** `loop()` contains no `delay()`. Sensing, beeping and printing are
   independently scheduled with `millis()`, so serial commands are always
   responsive.

## Serial commands

Open the Serial Monitor at **9600 baud**, line ending **Newline**.

| Command | Effect |
|---|---|
| `T30` or `30` | Set alert threshold to 30 cm (persisted to EEPROM) |
| `?` | Print current distance, threshold and alert state |

Sample output:

```
Distance: 42.3 cm  |  Threshold: 25 cm  |  Status: clear
Distance: 18.7 cm  |  Threshold: 25 cm  |  Status: ALERT - OBJECT TOO CLOSE
>> Threshold set to 15 cm (saved)
```

## Running the simulation

Open the Wokwi link above, press the green play button, click the HC-SR04 and
drag its distance slider to move a virtual object towards the sensor.

## Possible extensions

- Add an OLED or 16x2 LCD for a standalone readout with no PC attached
- Median filter replaced by a Kalman filter for a moving target
- Second sensor for left/right obstacle direction, as used on mobile robots
