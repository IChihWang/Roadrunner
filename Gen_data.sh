#!/bin/bash

#365 369 399 696 699 711 799 996 7 666

for j in $(seq 0 1 100)
do
  for i in $(seq 0.1 0.1 1.0)
  do
      echo $j $i
      python new_runner.py $i $j
  done
done 
