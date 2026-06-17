# A more reluctant and stable version for slower OT slaves
Using the original edge-ISR instead of sampling using a timer-ISR

I used Ihor Melnyk's libraries for some years using an ESP8266 and Arduino sketch without any issues. There are 2 DS18b20 connected using one-wire and a display connected via SPI, WiFi enabled, a webserver and HA connectivity, all working fine for years!
https://github.com/ihormelnyk/opentherm_library

However I migrated all my IoT devices from Arduino to ESPHome and started to use the ESPHome core component of opentherm.

**ESPHome core component**

Using the ESPHome core component I got several warnings per second, indicating that it was missing a level transition in the 1000us timeframe.

[20:16:45.778][W][opentherm:373]: Protocol error occured while receiving response: NO_TRANSITION

[20:16:46.415][W][opentherm:373]: Protocol error occured while receiving response: NO_TRANSITION

[20:16:46.591][W][opentherm:373]: Protocol error occured while receiving response: NO_TRANSITION

The core is using a timer interrupt to sample the PIN level, running on 5kHz, which is one sample each 200us and therefore 2.5 samples per half-bit (500us). However when the boiler communication is not exactly 1ms (1kHz) then there is a drift between the ESP and boiler clock resuling in sync issues. The core is expecting a level shift at least every 5th sample (1ms) but when the boiler is running a bit slower this may be noted in the 6th sample.
Instead of trying to correct this, I itegrated the original code of Ihor

**This component**

Using an edge-ISR there is no drift issue as each bit will have 1 or 2 level shifts and re-sync the ESP timers. I still have protocol warnings (caused by noise) but only once every 3-4 minutes

21:51:30	[W]	[opentherm:379]	Protocol error occured while receiving response: PARITY_ERROR

21:54:52	[W]	[opentherm:379]	Protocol error occured while receiving response: INVALID_STOP_BIT

# The drifting issue
Now the explanation for why there are so many NO_TRANSITION errors. This requires understanding of two interacting problems:

**Problem 1: Phase deviation in frame-start detection**

The LISTEN mode polling works as follows (from the code, lines 135–147):

```cpp
// LISTEN mode: timer ISR polls pin every 200µs
if (value) {  // pin HIGH = rising edge seen
    arg->read_();  // → clock_=1, capture_=1, mode=READ
}
```

The timer is already running during LISTEN. It doesn't detect the rising edge of the start bit at the exact moment, but at the next ISR — with a maximum delay of 200µs. The READ-mode samples then simply continue on the same 200µs grid as LISTEN, without a phase restart.

Result: the position of the READ samples relative to the start of the frame is random, somewhere within [0, 200µs].

**Problem 2: Phase deviation accumulates over the frame**

The boiler has its own clock (~1kHz). The ESP32 timer runs at 5kHz (200µs). These clocks are independent. Per bit, the phase drifts by `(T_boiler - 1000µs)`.

With a boiler that's 1% off (990Hz → 1010µs/bit):

* After 34 bits: 34 × 10µs = 340µs drift = 1.7 timer ticks

Combined with the initial phase uncertainty of ±200µs: total phase deviation up to ~540µs.

**How NO_TRANSITION results from this**

After a boundary transition, the state machine sets `clock_=1` and expects the mid-bit within 3 timer ticks (600µs). With a 540µs phase deviation, the mid-bit can now arrive at tick 3 or tick 4:

```
Boundary detected (clock_=1, capture_=1):
  tick+1: capture_ = 3
  tick+2: capture_ = 7
  tick+3: capture_ = 15 = 0xF     ← boundary
  tick+4: transition detected? → capture_ = 31, 31 > 0xF → NO_TRANSITION!
```

The mid-bit does arrive, but 200µs "too late" due to phase accumulation. The ISR only sees the transition at tick+4, causing `capture_ > 0xF` (= 31 > 15 = TRUE) — ERROR.

# Usage
The rest of the code is unchanged and therefore the usage is like the core esphome component

