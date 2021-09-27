source extern/laks/gdb_plugins/mmio.py
source extern/laks/gdb_plugins/rblog.py


set $ITMBASE=0xE0000000
set $DWTBASE=0xE0001000
set $FPBBASE=0xE0002000
set $SCBBASE=0xE000ED00
set $DCBBASE=0xE000EDF0
set $TPIUBASE=0xE0040000

# GDB macros are kinda shitty really.  doing proper python ones are "nicer" as you can control completions even.
define dwt_pc_sampling
  if $argc == 0
    set $enabled = *($DWTBASE) & (1<<12)
    set $slow = *($DWTBASE) & (1<<9)
    if $enabled
      if $slow
        printf "DWT based PC sampling is enabled at cyccnt/1024\n"
      else
        printf "DWT based PC sampling is enabled at cyccnt/64\n"
      end
    else
      printf "DWT based PC sampling is DISABLED\n"
    end
  else
    set *($DCBBASE+0xC) |= 0x1000000
    if ($arg0==1)
      set $speed = 64
      if ($argc == 2)
        if $arg1 == 1024
          set $speed = 1024
          set *($DWTBASE) |= (1<<9)
        else
          set *($DWTBASE) &= ~(1<<9)
        end
      end
      printf "sampling at cyccnt/%d\n", $speed
      set *($DWTBASE) |= (1<<12)
    else
      printf "Disabled DWT based PC sampling\n"
      set *($DWTBASE) &= ~(1<<12)
    end
  end
end
document dwt_pc_sampling
dwt_pc_sampling [<1|0> [64|1024]] Turn on/off, or check PC sampling via the DWT unit.
If you're turning it on, you can optionally pass the tap to use, 1024 or 64.
The 1024 tap is slower, but less likly to overrun.  Compare and combine with dwt_post_preset
end

define dwt_post_preset
  if $argc == 0
    set $val = ((*($DWTBASE+0) >> 1) & 0xf)
    printf "DWT.POSTPRESET = %d(%#x)\n", $val, $val
  else
  if (($arg0<0) || ($arg0>15))
    printf "Argument out of range <0..15>: %d(%#x)\n", $arg0, $arg0
    help dwt_post_preset
  else
    # out of scope, IMO, this is setting DBGMCU.TRCENA, but it's "safe"
    set *($DCBBASE+0xC) |= 0x1000000
    set *($DWTBASE) &= ~(0x0f<<1)    
    set *($DWTBASE) |= (($arg0&0x0f)<<1)
  end
  end
end
document dwt_post_preset
dwt_post_preset <0..15> Sets the reload value for the POSTCNT counter
In conjunction with the cycle count tap selected with dwt_pc_sampling,
this gives you a relatively wide range of sampling speeds.  Lower numbers are faster.
end
