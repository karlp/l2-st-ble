## bluetooth+laks+freertos

Using laks with freertos, with the STM32_WPAN bluetooth layer.
Targets an STM32WB, targetting extreme low power.

Original base demo project is: https://github.com/karlp/l2-simple

## State:
It works, but it's pretty rough.  You get LAKS for hw access,
freertos for your tasks, and ST's APIs as per AN5289 for the middleware.

No ST HAL code is included. No ST LL code is included.

Many chunks of code are simply straight out of ST demos though.

Two apps are built currently, demostrating what code is boilerplate and shared,
and what is "app" specific.

app-hrstm: ~equivalent to the ST BLE_HeartRateFreeRTOS demo app.
app-kustom1: does some custom advertising stuff, perhaps not very interesting

## Power consumption
At present, it uses stop2, with a LPTIMER tickless idle implementation from 
https://github.com/jefftenney/LPTIM-Tick and modified for c++ and laks.

Consumption is measured using after unplugging/replugging the nucleo board, to 
ensure debug is unpowered, as well as removing the VDD/T_VDD and SWDIO jumpers
from JP5.  (ST notes recommend removing all of them, but a) there's no need 
and b) aintnobodygottimeforthat.gif

I'm just eyeballing a meter here, so this is not super science.

### app-hrstm
About 200-300uAish while delivering notifications over bluetooth.
about 200uA while connected, but not notifications
about 270uA while in "fast advertising mode" for a few seconds after disconnecting
about 3uA while in "low power advertising"

### app-kustom1
About 200uA in fast advertising
about 3uA in slow advertising
about 200uA while connected


## TODO
* 99% sure that we don't need separate processes for shci and hci, just use flags and proper rtos features.
* be more "c++"  (code is very much C style, but it works for me...)
