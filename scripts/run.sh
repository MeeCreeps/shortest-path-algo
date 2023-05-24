set -ex 



#!/bin/bash

COMMAND="./scripts/exec.sh"

data=("ny" "bay" "fla" "usa")
data2=("col" "cal" "e" "ctr")
algo=("ch" "h2h" "phl")

# for a in "${algo[@]}"
# do 
#     for d in "${data[@]}"
#     do 
#         ${COMMAND} process ${a} ${d} 
#         wait
#     done
# done


for d in "${data[@]}"
do 
    ${COMMAND} process phl ${d} 
    wait
done

# generate 
for d in "${data[@]}"
do 
    ${COMMAND} generate ${d} 10000
    wait
done


for a in "${algo[@]}"
do 
    for d in "${data2[@]}"
    do 
        ${COMMAND} process ${a} ${d} 
        wait
    done
done




for d in "${data2[@]}"
do 
    ${COMMAND} generate ${d} 10000
    wait
done