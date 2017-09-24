#!/bin/bash

task(){
   PORT=$(((RANDOM<<15)|RANDOM))
   ./basicGrasp ../model/BHAM /home/rusi/Dropbox visualOff $PORT
}

open_sem(){
    mkfifo pipe-$$
    exec 3<>pipe-$$
    rm pipe-$$
    local i=$1
    for((;i>0;i--)); do
        printf %s 000 >&3
    done
}
run_with_lock(){
    local x
    read -u 3 -n 3 x && ((0==x)) || exit $x
    (
    "$@" 
    printf '%.3d' $? >&3
    )&
}

# Delete temp files                                                                                                                                                                                     
rm -rf ./tmp
rm -rf ../model/BHAM/*_include*.xml
rm -rf ../model/BHAM/*_Test.xml

N=5
open_sem $N
while true; do
    run_with_lock task
done 
wait
