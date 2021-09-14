env = SConscript('laks/build/env.py')

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
	#CCFLAGS = Split('-fno-inline'),
	CPPPATH = Dir('fmt/include'),
	#LIB_SOURCES = Glob('fmt/src/format.cc'),
	#CPPDEFINES = ['FMT_STATIC_THOUSANDS_SEPARATOR', 'FMT_EXCEPTIONS=0'],
)

env.Firmware('main.elf', ['main.cpp'])
#env.Firmware('main.elf', ['main.cpp', 'start.cpp'])

#env.Firmware('exc_test.elf', ['exc_test.cpp'])

#env.Command('main.S', 'main.o', '${TOOLCHAIN}objdump -d $SOURCE -l -C > $TARGET')

#env.Firmware('hello.elf', ['hello.cpp', 'start.cpp'])
#env.Firmware('main.elf', ['hello_f7.cpp'])

