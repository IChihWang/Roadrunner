#!/bin/bash

#365 369 399 696 699 711 799 996 7 666

rm result/*

for i in 0.6 0.65 0.7 0.75 0.8
do
  for j in $(seq 80 1 99)
  do
    for k in 0
    do
      echo $j $i $k
      python3 main.py $i $j $k T      
      python3 main.py $i $j $k F      
    done
  done
done 
