# Cell Module for ATTINY841

## LEDs
### Green LED
- blinks if communication is received
- remains lit for the next 10 communication cycles, if the cell module received a identify request

### Blue LED
- blinks twice if the WDT (Watchdog) is triggerd. Triggered every 8 seconds if no communication received.

### Red LED
- is on with bypass mode
- blinks if bypass in PWM mode
