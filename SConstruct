#!python
import os.path

env = SConscript('extern/laks/build/env.py')
#env.SelectMCU('stm32f746ng')
#env.SelectMCU('stm32f407vg')
#env.SelectMCU('stm32f427vi')
#env.SelectMCU('gd32vf103cb')
env.SelectMCU('stm32wb55rg')
#env.SelectMCU('stm32f303vc')
env.Replace(LINK_SCRIPT = "src/generated-st-ble.ld")

# Gross hack to make cmsis device includes work.  TODO - optionally add in selectMCU?
env.Append(CPPDEFINES = ['STM32WB55xx'])

env.SetOption("num_jobs", 8) # TODO - get this from the system

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
        FREERTOS = "#extern/freertos",
        FREERTOS_PORT = "#extern/freertos/portable/GCC/%s" % freertos_arch.get(env["PLATFORM_SPEC"]["meta"]["cpu"], "UNKNOWN_FREERTOS_ARCH"),
        )

env.Append(
	CPPPATH = [
            "${FREERTOS}/include",
            "${FREERTOS_PORT}",
            "#src",
            ],
)

sources_freertos = [os.path.join("${FREERTOS}/", x) for x in Split("list.c queue.c tasks.c timers.c")]
sources_freertos += ["${FREERTOS_PORT}/port.c"]
sources_freertos += ["${FREERTOS}/portable/MemMang/heap_1.c"]

####### CMSIS-DSP
env.SetDefault(CMSIS = "#extern/cmsis/CMSIS") # SetDefault allows overriding this from env vars
env.Append(CPPPATH = ["${CMSIS}/Core/Include", "${CMSIS}/DSP/Include"])

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
#	"CommonTables",
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
# Don't clean this library, we aren't ever actually editing it.
# Note, this still does the right thing if you _do_ edit the dsp source files.
if not env.GetOption('clean'):
	env.StaticLibrary("${CMSIS}/cmsisdsp", sources_dsp, CPPPATH=["${CMSIS}/Core/Include", "${CMSIS}/DSP/Include", "${CMSIS}/DSP/PrivateInclude"])
######


#### STM32_WPAN hacking...
env.SetDefault(WPAN = "#extern/stcube/Middlewares/ST/STM32_WPAN") # SetDefault allows overriding this from env vars
# ST likes inserting _lots_ of paths and using no prefixes, so be it...
env.Append(CPPPATH = [
    "${WPAN}",
    "${WPAN}/ble",
    "${WPAN}/ble/core",
    "${WPAN}/ble/core/auto",
    "${WPAN}/ble/core/template",
    "${WPAN}/ble/svc/Inc",
    "${WPAN}/interface/patterns/ble_thread",
    "${WPAN}/interface/patterns/ble_thread/tl",
    "${WPAN}/interface/patterns/ble_thread/shci",
    "${WPAN}/utilities",
])
env.SetDefault(CMSIS_ST = "#extern/stcube/Drivers/CMSIS/Device/ST") # SetDefault allows overriding this from env vars
env.Append(CPPPATH = [
    "${CMSIS_ST}/STM32WBxx/Include",
])
sources_wpan = [
    Glob("%s/ble/core/auto/*.c" % env["WPAN"]),
    Glob("%s/ble/core/template/*.c" % env["WPAN"]),
    Glob("%s/interface/patterns/ble_thread/shci/*.c" % env["WPAN"]),
    "%s/interface/patterns/ble_thread/tl/tl_mbox.c" % env["WPAN"],
    "%s/interface/patterns/ble_thread/tl/hci_tl.c" % env["WPAN"],
    "%s/interface/patterns/ble_thread/tl/hci_tl_if.c" % env["WPAN"],
    "%s/interface/patterns/ble_thread/tl/shci_tl.c" % env["WPAN"],
    "%s/interface/patterns/ble_thread/tl/shci_tl_if.c" % env["WPAN"],
    "%s/utilities/stm_list.c" % env["WPAN"],
    "%s/utilities/stm_queue.c" % env["WPAN"],
    # deliberately avoid dbg_trace, we're not using st hal stuff here!
    ]
# You'll want to pick/choose from the following...
wpan_services = [
#    "bas.c",
#    "bls.c",
#    "bvopus_service_stm.c",
#    "crs_stm.c",
    "dis.c",
#    "eds_stm.c",
#    "hids.c",
#    "hrs.c",
#    "hts.c",
#    "ias.c",
#    "lls.c",
#    "mesh.c",
#    "motenv_stm.c",
#    "opus_interface_stm.c",
#    "otas_stm.c",
#    "p2p_stm.c",
    "svc_ctl.c",
#    "template_stm.c",
#    "tps.c",
#    "zdd.c",
]

sources_wpan += [os.path.join(env["WPAN"], "ble/svc/Src/", x) for x in wpan_services]
# ST has lots of format errors in their code, and there's not much we can do about it...
env_wpan = env.Clone()
env_wpan.Append(CCFLAGS = ["-Wno-format"])
objs_wpan = [env_wpan.Object(f) for f in sources_wpan]

###########

env.Append(LIBS = "cmsisdsp", LIBPATH="${CMSIS}")
sources_app = ['main.cpp', 'analog.cpp', 'syszyp.cpp', 't_ble.cpp', 'dis_app.c', 'tgt_hw_ipcc.cpp']

fsources = [env.Object(f) for f in Flatten([  [os.path.join('src', x) for x in sources_app] + sources_freertos + env['LIB_SOURCES']])]
fw = env.Program("main.elf", fsources + objs_wpan)
env.Depends(fw, env['LINK_SCRIPT'])
