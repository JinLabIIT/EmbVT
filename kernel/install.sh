#!/bin/bash

function usage() {
        echo "Usage: $0 target"
        echo "  target: either [dilation] or [freeze], default=[both]"
        exit 1
}

DST=~/build_kernel
TARGET=$1

if [ -e $DST ]; then
        echo "Clean up previous build"
        sudo rm -rf $DST
fi

if [ ! -e bananapi-3.4.zip ]; then
        wget https://github.com/Bananian/linux-bananapi/archive/bananapi-3.4.zip
fi
echo "Step 0. Unpack kernel source"
unzip linux-bananapi-bananapi-3.4.zip
mv linux-bananapi-bananapi-3.4 $DST

# generate patch file
PATCH=VirtualTime.patch
echo "        Patch file written to ${PATCH}"
diff -rup $DST ./ > ${PATCH}
echo ""

FILES="kernel/fork.c		        \
kernel/time.c			        \
kernel/posix-timers.c                   \
kernel/time/timekeeping.c	        \
include/linux/sched.h		        \
include/linux/init_task.h	        \
include/linux/time.h		        \
include/linux/ktime.h                   \
include/linux/timekeeper_internal.h     \
fs/proc/base.c			        \
include/net/sch_generic.h	        \
Makefile		                \
build_all.sh"
#net_dilation/core/dev.c                \

echo "Step 1. Transfer modified kernel source"
for f in $FILES; do
        cp -v $f $DST/$f
done

if [ "dilation" = $TARGET ]; then
        cp -v net_dilation/sched/sch_generic.c $DST/net/sched/sch_generic.c
        cp -v net_dilation/sched/sch_htb.c $DST/net/sched/sch_htb.c
elif [ "freeze" = $TARGET ]; then
        cp -v net_freeze/sched/sch_generic.c $DST/net/sched/sch_generic.c
        cp -v net_freeze/sched/sch_htb.c $DST/net/sched/sch_htb.c
elif [ "both" = $TARGET ]; then
        cp -v net_both/sched/sch_generic.c $DST/net/sched/sch_generic.c
        cp -v net_both/sched/sch_htb.c $DST/net/sched/sch_htb.c
else
        usage
fi

echo "Step 2. Build new kernel"
cd $DST
sudo ./build_all.sh
