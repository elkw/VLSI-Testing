#!/bin/bash
### define ckt name
ckt_name=("17" "432" "499" "880" "1355" "2670" "3540" "6288" "7552")

### define 
DIR_list=("golden" "output")

cd ..

for DIR in "${DIR_list[@]}"
do
    if [ -d "${DIR}" ]; then
        ### Take action if $DIR exists ###
        echo "${DIR} directory exists..."
    else
        ###  Control will jump here if $DIR does NOT exists ###
        echo "${DIR} not found. Create directory."
        mkdir ${DIR}
    fi
done

# generate golden output
cd ./bin
for i in "${ckt_name[@]}"
do
    ./golden_tdfsim -tdfsim ../tdf_patterns/c${i}.pat ../sample_circuits/c${i}.ckt > ../golden/c${i}.ptn
done
cd ../src
make
for i in "${ckt_name[@]}"
do
    ./atpg -tdfsim ../tdf_patterns/c${i}.pat ../sample_circuits/c${i}.ckt > ../output/c${i}.ptn
    diff ../golden/c${i}.ptn ../output/c${i}.ptn
done
