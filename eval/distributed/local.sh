#!/bin/bash
#
# Author: Christopher Hannon
# Date: 10-17-18
#
###############################
#
#
#
#  Usage: ./local [numProcs] [saveDir] 
#
#  Number of procs is $1
#  Log folder is $2

if [ "$#" -ne 2 ]; then
    echo "Usage: ./local [numProcs] [saveDir] "
    exit
fi
echo $$ > "./${0}.pid"
numPids=$1
p2d=$2

trap 'getFPT' 10
trap 'finish' 12
trap 'cleanup' 1 2 3 6

cleanup()
{
    echo "catching exit ..."
    rm -f "./${0}.pid"
    exit 1
}

getFPT()
{
    echo "getting FPTs ..."
    for q in "${pids[@]}"
    do
      	# removes ns\n and replaces with , save to file
	cat /proc/${q}/fpt | tr 'ns\n' ' , ' >> /home/emb-vt/eval/distributed/skew/${p2d}/skew_${i}.log
    done
    echo ' ' >> /home/emb-vt/eval/distributed/skew/${p2d}/skew_${i}.log
}

finish()
{
    #######################
    ### exit experiment ###
    #######################
    #
    # saving log files
    #
    echo "saving log files to /home/emb-vt/eval/distributed/[skew\overhead]/${p2d}/[skew\overhead]_${i}.log"
    # 
    kill -9 ${pids[@]}
    #
    # save dmesg
    dmesg > /home/emb-vt/eval/distributed/overhead/${p2d}/overhead_${i}.log

    #
    # Cleanup
    # clear pids
    for x in `seq 1 16`;
    do
	xx=$x
	while [[ ${#xx} -lt 2 ]] ; do
	    xx="0${xx}"
	done
	
	echo "0" > /sys/vt/VT7/pid_${xx}
    done
    echo "------------------------------------------------------------------------------------------------------------------"
    echo "thank you and goodbye"
    echo "------------------------------------------------------------------------------------------------------------------"
    rm -f "./${0}.pid"
    exit 1
}


# main
#
for i in $1 #2 4 8 16
do
    #############
    #  Logging  #
    #############
    if [ -d "./overhead/${p2d}" ]; then
	# Control will enter here if $DIRECTORY exists.
	echo "overhead directory exists already, exiting..."
	rm "./${0}.pid"
	exit
    fi
    if [ -d "./skew/${p2d}" ]; then
	# Control will enter here if $DIRECTORY exists.
	echo "skew directory exists already, exiting..."
	rm "./${0}.pid"
	exit
    fi
    mkdir "./overhead/${p2d}"
    mkdir "./skew/${p2d}"
    sleep 2
    #
    ######################
    # rebuilding kmodule #
    ######################
    rmmod vtgpio
    cd /home/emb-vt/src
    make
    insmod vtgpio.ko
    cd /home/emb-vt/eval/distributed
    # clear dmesg
    sleep 5
    dmesg --clear
    ########################
    ### setup experiment ###
    ########################
    echo "setting up experiment with ${i} processes"
    # declare array
    declare -a pids
    #
    # z = upperbound
    z=$(($i-1))
    #
    # start sleeping procs
    for j in `seq 0 $z`; do
	#
	# get pid variable to be double digits (+1)
	jj=$(($j+1))
	# make jj 2 digits for the pid_01-16 variable
	while [[ ${#jj} -lt 2 ]] ; do
	    jj="0${jj}"
	done
	#
	# add sleeping procs to VT
	sleep 25000 &
	pids[j]=$!
	pid_path="/sys/vt/VT7/pid_${jj}"
	echo $! > ${pid_path}
	echo $!
    done
    ######################
    ### run experiment ###
    ######################
    echo "entering wait loop"
    #
    # start all procs
    sleep 1
    kill -CONT ${pids[*]}
    #
    while kill -0 ${pids[0]} >/dev/null 2>&1
    do
	
	sleep 1
    done
    wait ${pids[0]}
done