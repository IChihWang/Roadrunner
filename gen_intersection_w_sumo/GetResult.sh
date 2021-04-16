#!/bin/bash

#365 369 399 696 699 711 799 996 7 666

for j in 123 777 333 147 66 99 19 12 250 360
do
  for i in $(seq 0.1 0.1 1.0)
  do
    for k in $(seq 0 1 2)
    do
      echo $j $i $k
      python new_runner.py $i $j $k >> result/$i.$k.txt
    done
  done
done 
