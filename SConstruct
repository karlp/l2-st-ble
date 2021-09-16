#!python
import os.path

env = SConscript('laks/build/env.py')
env.SetDefault(
        FREERTOS = "#freertos",
        FREERTOS_PORT = "#freertos/portable/GCC/ARM_CM4F",  # TODO - pull it out of selectmcu's result? hahaha
        )

#env.SelectMCU('stm32f746ng')
#env.SelectMCU('stm32f407vg')
#env.SelectMCU('stm32f427vi')
#env.SelectMCU('gd32vf103cb')
env.SelectMCU('stm32wb55rg')

env.Append(
	CXXFLAGS = Split('-fcoroutines -Wno-volatile'),
	LINKFLAGS = Split('--specs=nano.specs'),
)

env.Append(
	CPPPATH = [
            "${FREERTOS}/include",
            "${FREERTOS_PORT}",
            "#.",
            ],
)

sources_freertos = [os.path.join("${FREERTOS}/", x) for x in Split("list.c queue.c tasks.c timers.c")]
sources_freertos += ["${FREERTOS_PORT}/port.c"]
sources_freertos += ["${FREERTOS}/portable/MemMang/heap_1.c"]

env.Firmware('main.elf', ['main.cpp'] + sources_freertos)
