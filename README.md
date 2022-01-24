## bluetooth+laks+freertos

Using laks with freertos, with the STM32_WPAN bluetooth layer.
Targets an STM32WB, targetting extreme low power.

Original base demo project is: https://github.com/karlp/l2-simple

## State:
It works, but all power saving is disabled.  You get LAKS for hw access,
freertos for your tasks, and ST's APIs as per AN5289 for the middleware.

No ST HAL code is included. No ST LL code is included.

Many chunks of code are simply straight out of ST demos though.


## TODO
* low power, obviously.

hci_cmd_resp_wait and hci_cmd_resp_release should probably be implemented using rtos functions?
the default weak impls are busy polling?

Split up the code a lot more into "app" and "glue"
