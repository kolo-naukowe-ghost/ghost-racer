#!/bin/bash

# vars
root_filename=".ghost-racer-root"
cwd=$(pwd)

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
    xterm -T main -ls -xrm 'XTerm*selectToClipboard:true' -e "/bin/bash -c \"cd $cwd;source devel/setup.bash ; roslaunch autonomous_driving_world spawn_major_in_competition_track_D1_D2_B1_P1.launch\"" &
    xterm -T video_reader -ls -xrm 'XTerm*selectToClipboard:true' -e "/bin/bash -c \"${cwd}/devel/lib/stub_video_reader/stub_video_reader_node \"" &
}


if [ -f $root_filename ]; then
if [[ "$@" == *"--clean"* ]]; then
        clean
    fi
    build && run
else
    echo "You can only run ghost-car from the directory containing file $root_filename"
fi