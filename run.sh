#!/bin/bash

# vars
root_filename=".ghost-racer-root"
cwd=$(pwd)

function wait_for_program() {
    [ -z $1 ] && echo "Program name not specified" && return 1
    [ -z $2 ] && echo "Timeout not specified" && return 1

    echo "Waiting for program '$1', timeout $2 sec"
    for times in $( seq 1 $2 )
    do
        echo "Checking time $times, name $1"
        if pgrep "$1" > /dev/null ; then
            return 0
        fi
        sleep 1
    done

    return 1
}

clean () {
    echo "Removing build and devel"
    rm -rf build devel
}

build () {
    echo "Building"
    catkin_make
}

run () {
    echo "Running from directory $cwd"
    xterm -T main -ls -xrm 'XTerm*selectToClipboard:true' -e "/bin/bash -c \"cd $cwd;source devel/setup.bash ; roslaunch autonomous_driving_world spawn_conde_in_competition_track_D1_D2_B1_P1.launch\"" &
    if wait_for_program gzserver 10 ; then
        echo "Starting agent"
        xterm -hold -T agent -ls -xrm 'XTerm*selectToClipboard:true' -e "/bin/bash -c \"cd $cwd; source devel/setup.bash; rosrun environment agent.py\"" &
    else
        echo "Failed to open agent, quitting ghost-racer"
        ./kill_ros.sh
    fi
}


if [ -f $root_filename ]; then
    build && run
else
    echo "You can only run ghost-car from the directory containing file $root_filename"
fi