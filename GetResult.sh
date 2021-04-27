#!/bin/bash

#365 369 399 696 699 711 799 996 7 666

for j in 6 7 16 17 26 27
do
  for i in 0.5375 0.0625 0.3125
  do
    for k in 0
      do
      for prob in $(seq 0.1 0.1 0.9)
      do	
      	echo $j $i $k
      	python3 main.py $i $j $k T $prob
      done
    done
  done
done 
