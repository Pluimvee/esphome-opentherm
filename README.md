# A more stable version of the Opentherm component❗
Using the original edge-ISR instead of samplig the pin using a timer-ISR

I used Ihor Melnyk's libraries for some years using an ESP8266 and Arduino sketch without any issues. There are 2 DS18b20 connected using one-wire and a display connected via SPI, WiFi enabled, a webserver and HA connectivity, all working fine for years!
https://github.com/ihormelnyk/opentherm_library

However I migrated all my IoT devices from Arduino to ESPHome and started to use the ESPHome core component of opentherm.

### ESPHome core component 
Using the ESPHome core component I got several warnings per second, indicating that it was missing a level transition in the 1000us timeframe.

[20:16:45.778][W][opentherm:373]: Protocol error occured while receiving response: NO_TRANSITION

[20:16:46.415][W][opentherm:373]: Protocol error occured while receiving response: NO_TRANSITION

[20:16:46.591][W][opentherm:373]: Protocol error occured while receiving response: NO_TRANSITION

The core is using a timer interrupt to sample the PIN level, running on 5kHz, which is one sample each 200us and therefore 2.5 samples per half-bit (500us). However when the boiler communication is not exactly 1ms (1kHz) then there is a drift between the ESP and boiler clock which can cause thiese kind of warnings/errors. The core is expecting a level shift at least every 5th sample (1ms) but when the boiler is running a bit slower this may be noted the 6th sample.
Instead of trying to correct this, I went to the original code of Ihor

### This component
Using an edge-ISR there is no drift issue as each bit will have 1 or 2 level shifts and re-sync the ESP timers. I still have protocol warnings (caused by noise) but only once every 3-4 minutes

21:51:30	[W]	[opentherm:379]	Protocol error occured while receiving response: PARITY_ERROR

21:54:52	[W]	[opentherm:379]	Protocol error occured while receiving response: INVALID_STOP_BIT

# OpenTherm ESPHome core
The rest of the code is unchanged and therefore the usage is like the core esphome component


- [Original Arthur Rump's repository](https://github.com/arthurrump/esphome-opentherm)
- [arduino-opentherm project by Jiří Praus](https://github.com/jpraus/arduino-opentherm)

There is also my blog post with more background details and reasoning for automating an OpenTherm boiler with ESPHome:

- [OpenTherm thermostat with ESPHome and Home Assistant](https://olegtarasov.me/opentherm-thermostat-esphome/)
