## Fans plugin

Under development. Adds two M-codes for controlling fans.

* `M106 <P->` turns fan on. The optional P-word specifies the fan, if not supplied fan 0 is turned on.
* `M107 <P->` turns fan off. The optional P-word specifies the fan, if not supplied fan 0 is turned off.

The new realtime command `0x8A` can also be used to toggle fan 0 on/off even when a G-code program is running.

Add a line with

`#define FANS_ENABLE <n>`

to _my_machine.h_ to enable `<n>` fans, e.g. `#define FANS_ENABLE 1` for one.

If the driver supports mapping of port number to fan the following $-settings, depending on number of fans configured, are made available:

`$386` - for mapping aux port to Fan 0.  
`$387` - for mapping aux port to Fan 1.  
`$388` - for mapping aux port to Fan 2.  
`$389` - for mapping aux port to Fan 3.

Use the `$pins` command to see which port/pin is currently assigned.  
__NOTE:__ A hard reset is required after changing port to fan mappings.  

Fans can be linked to the spindle enable command, thus turning them automatically on and off depending on the spindle state.  
__NOTE__: If a fan is turned on by `M106` \(or the new real time command\) before enabling the spindle it will _not_ be turned off automatically when the spindle is stopped.

`$483` - bits for linking specific fans to spindle enable.

Fan 0 can be configured be turned off automatically on program completion, or spindle disable if linked, after a configurable delay.  

`$480` - number of minutes to delay automatic turnoff of fan 0.   
__NOTE__: If set to 0 fan 0 is not automatically turned off by program end and is turned off immediately if linked to spindle enable.

---

__NOTE:__ The M-codes are adopted from [Marlin specifications](https://marlinfw.org/docs/gcode/M106.html) \(with fewer parameter values supported\).

Dependencies:

Driver must have at least `<n>` [ioports port](../../templates/ioports.c) output\(s\) available.
Requires grblHAL build 20210629 or later and the fans plugin added to the source tree.

---
2023-01-26
