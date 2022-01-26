## bluetooth+laks+freertos

Using laks with freertos, with the STM32_WPAN bluetooth layer.
Targets an STM32WB, targetting extreme low power.

Original base demo project is: https://github.com/karlp/l2-simple

## State:
It works, but all power saving is disabled.  You get LAKS for hw access,
freertos for your tasks, and ST's APIs as per AN5289 for the middleware.

No ST HAL code is included. No ST LL code is included.

Many chunks of code are simply straight out of ST demos though.

Two apps are built currently, demostrating what code is boilerplate and shared,
and what is "app" specific.

app-hrstm: ~equivalent to the ST BLE_HeartRateFreeRTOS demo app.
app-kustom1: does some custom advertising stuff, perhaps not very interesting

## TODO
* low power, obviously.
* hci_cmd_resp_wait and hci_cmd_resp_release should probably be implemented using rtos functions?
the default weak impls are busy polling?
same for shci_cmd_resp_wait and shci_cmd_resp_release.  /yes, youi've already got the semaphores created for them, rip of ST here!
* 99% sure that we don't need separate processes for shci and hci, just use flags and proper rtos features.
* be more "c++"  (code is very much C style, but it works for me...)
