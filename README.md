# New Content

### Entry point
The entry point for all pausing happens **asyncronously** through `vtgpio_irq_handler`.

The entry point for all resuming happens **asyncronously** through `vtgpio_irq_handler_fall`

### Triggerering `vtgpio_irq_handler`
This function is caused when the hardware interrupt handler detects a **rising edge** on the associated pin 1. 

### Triggering `vtgpio_irq_handler_fall`
This function is caused when the hardware interrupt handler detects a **falling edge** on the associated pin 2. 

### Remote calling vs local calling

A signal caused by a remote host is treated the same as a local host. The User space process **NOT IN VT** must be responsible for resuming the virtual time system by writing `unfreeze` to `/sys/vt/VT7/mode` on the host that triggered the pause.

### Pausing and Resuming 

Triggering `vtgpio_irq_handler` and `vtgpio_irq_handler_fall` locally is done by:
1. write `freeze` to `/sys/vt/VT7/mode` This is the case in dynamic mode, on demand mode, or "at runtime" however we end up calling it.
1. if a period is written at the time of loading, the periodic callback triggers `vtgpio_irq_handler`. It is up to a (any) user space program to resume. **Note** this must be done by polling `/sys/vt/VT7/syncpause` 

### misc notes
If we are already paused, i.e., by realtime sync event (local or remote) -- the module does not distinguish (though it could) this function can not get called again until `vtgpio_irq_handler`. This is due to the rules of physics. However if temorally indistinguishable events happen at near-the-same time, the system wont call pause until all events are processed.


### Pause 
Pause only calls `sequential_io(FREEZE)` which performs 2 steps:

1. sends the `SIGSTOP` command to registered PIDS. 
1. captures time variables to use for clock offsets 

### Resume
Resume only calls `sequential_io(RESUME)` which performs 2 steps:

1. sends the `SIGCONT` command to registered PIDS. 
1. captures time variables to use for clock offsets 



# emb-vt
Distibuted Kernel Module and modified time keeping functions for enabling virtual time in distributed heterogeneous embedded linux environments

About: Banana Pi devices with 1GB RAM 1.2 Ghz 2 core processors and Banana Pi Routers for ethernet 1 gbps networking

Software Environment:

base directory for the project

cd /home/emb-vt 

src is the source for the kernel Module

kernel contains the source for the modified kernel 

eval contains the benchmarking code and various example scripts to test performance

if kernel is compiled with debug and function tracer environments then /debug can be mounted see EVAL / TEST sections

## Kernel Module

The Kernel Module is a wrapper to the virtual time kernel functionality. The module also is necessary for the distribution of the pause / unpause (freeze / unfreeze) across computers.

inside the src folder

`make` compiles the module 

`insmod vtgpio_test.ko &` loads the kernel module

`rmmod vtgpio_test` unloads the kernel module 

`lsmod` lists the kernel modules currently loaded

`dmesg` prints the kernel log  i.e.,  everything that has been written with `printk()`

`dmesg | tail` is useful or `dmesg | grep VT-GPIO`

`/sys/vt/VT7/` contains `mode`, `tdf`, `pid_01-16`

These file system entries are called kobjects or something like this, when the kernel module is loaded it creates these entries. There is a handler registered to each entry. When the value of an entry is modified, the assiciated handler is eventually run (pending OS scheduling). It should be possible to locate the handlers inside the kernel module source code. 

`mode` triggers the freeze and unfreeze routines inside the kernel module

by typing `echo $PID > /sys/vt/VT7/pid_01` you have entered a pid $PID into virtual time 

If you recieve an error can not open `/proc/$PID/dilation` in `dmesg` then likely the process does not exist or else it may be a lightweight thread which does not have file system entries for it.

try creating an infinite loop program in python with large sleeps (to keep the cpu free) as a dummy program

`cat /sys/vt/VT7/mode` should initially return `unfreeze` by typing `echo 'freeze' > /sys/vt/VT7/mode` you initiate the handler in the kernal module. to unfreeze just echo unfreeze.

tdf value stands for time dialation factor times 1000. In our case this value should just stay at the default of 1000. (changing it to 2000 would make the process think time is going 2x speed)


### Distributed part

In the case of multiple boards, there are two interrupt handlers, they are used for pause and resume ( just like /sys/vt/VT7/mode is) when the mode is changed on one board, the kernel module also sends out high signals on the pin that connects the boards. The interrupt handlers run the same routine as if their mode was changed locally. 

## Kernel modifications

the timekeeping library is modified to calculate the offset of the virtual clock to the wall clock time.

you can see the changes by running diff on the modified files vs the files found by default when installing the kernel source 

A good Description can be found in the virtual time by jiaqi Yan  paper in ACM PADS conference 2015 and in my DSSnet paper PADS conference 2016 in the implementation section 


Basically the kernel module is a wrapper that modifies the /proc/$PID/ file system and changes hardware interrupt pin voltages.


## runthrough

- app A is created
- app A is added to virtual time ( `echo $PID > /sys/vt/VT7/pid_01`)
- app A runs for a while
- app B decides that the virtual time processes should be paused ( `echo 'freeze' > /sys/vt/VT7/mode`)
- kernel module will call the handler (mode_store) function which will call freeze or unfreeze ( and change the pins for the distributed part -- board to board-- )
- the freeze/unfreeze function writes the proper attribute into the process attributes in the /proc/$PID/... file system which the kernel can then process
- the kernel module exits
- when the kernel reads the /proc/$PID/ entries ( I think when the OS scheduler schedules the proc) the kernel will freeze the app A
- App A is now frozen

- after resuming, if app A calls gettimeofday system call, it will see the wallclock time - the amount of time frozen for .

## Distributed algorithm stuff

TODO

pin arrangement

xxxxxxooooxxx
      
xxxxooooooxxx 

ooooooooooooo

grobooooooooo
