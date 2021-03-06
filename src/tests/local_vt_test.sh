#!/bin/bash
echo "kicking off processes in paused state"
./start_paused.sh ./A.py &
PID1=$!
./start_paused.sh ./B.py > vt_log.txt &
PID2=$!
echo $PID1
echo $PID2
## DILATE ##
echo "adding P2 to VT"
echo $PID1 > /sys/vt/VT7/pid_01
sleep 4
echo "starting processes"
kill -CONT $PID1
kill -CONT $PID2
while kill -0 $PID1 >/dev/null 2>&1
do
    echo "freezing";
    #START = 'date +"%T.%N"'
    #time
    #kill -STOP $PID1 
    echo "freeze" > /sys/vt/VT7/mode
    #END = date +"%T.%N"
    #echo Time taken to freeze: $((END - START))
    sleep 2
    echo "unfreezing"
    #START = date +"%T.%N"
    #time
    echo "unfreeze" > /sys/vt/VT7/mode
    #kill -CONT $PID1
    #END = date +"%T.%N"
    #echo Time taken to freeze: $((END - START))
    sleep 2
done
wait $PID1
wait $PID2
