#!python
import os.path

env = SConscript('laks/build/env.py')
#env.SelectMCU('stm32f746ng')
#env.SelectMCU('stm32f407vg')
#env.SelectMCU('stm32f427vi')
#env.SelectMCU('gd32vf103cb')
env.SelectMCU('stm32wb55rg')
#env.SelectMCU('stm32f303vc')

env.Append(
	CXXFLAGS = Split('-fcoroutines -Wno-volatile'),
	LINKFLAGS = Split('--specs=nano.specs'),
)

freertos_arch = {
	"cortex-m7f": "ARM_CM7/r0p1",
	"cortex-m4f": "ARM_CM4F",
	"cortex-m3": "ARM_CM3",
	"cortex-m0": "ARM_CM0",
	"cortex-m0+": "ARM_CM0",
	"rv32imac": "RISC-V",
}

env.SetDefault(
        FREERTOS = "#freertos",
        FREERTOS_PORT = "#freertos/portable/GCC/%s" % freertos_arch.get(env["PLATFORM_SPEC"]["meta"]["cpu"], "UNKNOWN_FREERTOS_ARCH"),
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

####### CMSIS-DSP
env.SetDefault(CMSIS = "#cmsis/CMSIS") # SetDefault allows overriding this from env vars
env.Append(CPPPATH = ["${CMSIS}/Core/Include", "${CMSIS}/DSP/Include", "${CMSIS}/DSP/PrivateInclude"])

# Provide defines and things for cmsis-dsp based on selected mcu...
dsp_opts = {
	"cortex-m7f": {"defs": ["CORTEXM", "ARMCM7_DP"]},
	"cortex-m4f": {"defs": ["CORTEXM", "ARMCM4_FP"]},
	"cortex-m3": {"defs": ["CORTEXM", "ARMCM3"]},
	"cortex-m0": {"defs": ["CORTEXM", "ARMCM0"]},
	"cortex-m0+": {"defs": ["CORTEXM", "ARMCM0P"]},
}
env.Append(CPPDEFINES = dsp_opts.get(env["PLATFORM_SPEC"]["meta"]["cpu"], {"defs": []})["defs"])

# Optional configuration flags you may wish to apply, uncomment as you see fit
dsp_flags = [
	#"ARM_MATH_LOOPUNROLL",
	#"ARM_MATH_ROUNDING",
	#"ARM_MATH_MATRIX_CHECK",
	#"ARM_MATH_AUTOVECTORIZE",
]
env.Append(CPPDEFINES = dsp_flags)

# Individually select what you like here
dsp_modules = [
#	"BayesFunctions",
	"TransformFunctions",
	"StatisticsFunctions",
#	"ComplexMathFunctions",
	"BasicMathFunctions",
#	"ControllerFunctions",
	"CommonTables",
	"FilteringFunctions",
#	"InterpolationFunctions",
#	"QuaternionMathFunctions",
#	"DistanceFunctions",
	"SupportFunctions",
#	"FastMathFunctions",
#	"MatrixFunctions",
#	"SVMFunctions",
]
sources_dsp = [Glob(os.path.join("%s/DSP/Source" % env["CMSIS"], sdir, "*.c")) for sdir in dsp_modules]
env.StaticLibrary("cmsisdsp", sources_dsp)
######

env.Append(LIBS = "cmsisdsp", LIBPATH=".")
env.Firmware('main.elf', ['main.cpp', 'syszyp.cpp'] + sources_freertos)
