# Cell Module for ATTINY841

## LEDs
### Green LED
- is on if a package is received
- blinks twice if the setup process is nearly complete
- keeps on for the next 10 Packages, if the CellModule received a identify package

### Blue LED
- blinks twice if the WDT (Watchdog) is triggerd. Can be triggert every 8 seconds
- blinks twice if the setup process is nearly complete
- blinks twice if the module wake up

### Red LED
- is on with bypass mode
- blinks if bypass in PWM mode
