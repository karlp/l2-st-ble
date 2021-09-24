## Purpose
laks v2 demo for stm32wb

This project does "nothing" other than setup a framework of a few tools.

* Laks for the c++ startup, peripheral register access.
  This is a _very_ thin layer, focused on functional code, that compiles to optimized code.
  I'd point you to the laks readme, but there isn't one...
  Laks implies the use of SCons.
* FreeRTOS because doing things in big main loops is so 80s.
  This project isn't a FreeRTOS demo, but it does show how to build FreeRTOS for
  a c++ project, with SCons.
* CMSIS-DSP.
  Again, the utility here is largely showing how to build it in SCons.

It is/was largely an exercise in how setup new peripherals in laks, and use them in
a from scratch project.  It does timer triggered multi channel ADC sampling,
DMA transferred to a buffer, with a FreeRTOS task using CMSIS-DSP to "filter" the data.
