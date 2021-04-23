#!/bin/bash

#365 369 399 696 699 711 799 996 7 666

rm result/*

for j in 4 5
do
  for i in $(seq 0.05 0.05 0.65)
  do
    for k in $(seq 0 1 3)
    do
      echo $j $i $k
      python3 main.py $i $j $k T
    done
  done
done 
