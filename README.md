## bluetooth+laks+freertos

Using laks with freertos, with the STM32_WPAN bluetooth layer.
Targets an STM32WB, targetting extreme low power.

Original base demo project is: https://github.com/karlp/l2-simple

## State:
It works, but all power saving is disabled.  You get LAKS for hw access,
freertos for your tasks, and ST's APIs as per AN5289 for the middleware.

No ST HAL code is included. No ST LL code is included.

Many chunks of code are simply straight out of ST demos though.
