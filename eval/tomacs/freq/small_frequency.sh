
rmmod vtgpio
cd /home/EmbVT/src
make
insmod vtgpio.ko period=0
cd /home/EmbVT/eval/tomacs/freq

sleep 250000 &
pid_path="/sys/vt/VT7/pid_01"
echo $! > $pid_path
cat /sys/vt/VT7/pid_01

sleep 5

for i in 1 0.5 0.25 0.125 0.0625 0.03125 0.015625; do
    dmesg --clear
    loop=0

    while [ $loop -lt 1024 ]; do
	echo $loop / 1024 $i / 1 0.5 0.25 0.125 0.0625 0.03125 0.015625
	let loop=loop+1
	sleep $i
	echo "freeze" > /sys/vt/VT7/mode
	sleep $i
	echo "unfreeze" > /sys/vt/VT7/mode
    done
    dmesg > /home/EmbVT/eval/tomacs/freq/data/freq_dyn_"${i}"
done

sleep 10

for i in 1 0.5 0.25 0.125 0.0625 0.03125 0.015625; do
    rmmod vtgpio
    cd /home/EmbVT/src
    insmod vtgpio.ko period=$i
    cd /home/EmbVT/eval/tomacs/freq
    sleep 3
    sleep 250000 &
    echo $! > /sys/vt/VT7/pid_01
    dmesg --clear
    loop=0

    while [ $loop -lt 1024 ]; do
	echo $loop / 1024 $i / 1 0.5 0.25 0.125 0.0625 0.03125 0.015625
	let loop=loop+1
	while [ $(cat /sys/vt/VT7/syncpause) -ne 1 ];
	do
	    sleep .01
	done
	#echo "freeze" > /sys/vt/VT7/mode
	sleep $i
	echo "unfreeze" > /sys/vt/VT7/mode
    done
    dmesg > /home/EmbVT/eval/tomacs/freq/data/freq_period_"${i}"
done
