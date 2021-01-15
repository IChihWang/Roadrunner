#!/bin/bash

for j in 123 777 333 147 66 #99 19 12 250 360 365 369 399 696 699 711 799 996 7 666
do
for i in $(seq 0.15 0.1 0.95)
do 
  echo $j 
  python new_runner1.py $i $j >> $j\_$i.txt

done
done