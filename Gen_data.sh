#!/bin/bash

#365 369 399 696 699 711 799 996 7 666

for j in $(seq 101 1 200)
do
  for i in $(seq 0.1 0.1 1.0)
  do
      echo $j $i
      python main.py $i $j
  done
done 
