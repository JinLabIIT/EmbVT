#for i in 1000 500 250 125 62 31 16; do
for i in 62 31 16; do
    rmmod vtgpio
    cd /home/EmbVT/src
    echo $i
    #make clean
    make
    insmod vtgpio.ko period=$i
    cd /home/EmbVT/eval/tomacs/freq
    sleep 3
    sleep 250000 &
    echo $! > /sys/vt/VT7/pid_01
    dmesg --clear
    loop=0
    echo "starting"

    while [ $loop -lt 1024 ]; do
	echo $loop / 1024  $i / 1 0.5 0.25 0.125 0.0625 0.03125 0.015625
	let loop=loop+1
	while [ $(cat /sys/vt/VT7/syncpause) -ne 1 ];
	do
	    echo "waiting"
	    sleep .2
	done
	#echo "freeze" > /sys/vt/VT7/mode
	sleep 0.1
	echo "unfreezing"
	echo "unfreeze" > /sys/vt/VT7/mode
	sleep 0.1
    done
    dmesg > /home/EmbVT/eval/tomacs/freq/data/freq_period_"${i}"
done
