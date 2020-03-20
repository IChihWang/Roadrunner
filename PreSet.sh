#!/bin/bash

rm -r advise_info
rm -r inter_info

mkdir advise_info
mkdir inter_info

cd gen_intersection
make

rm -r inter_data
rm -r advise_data
mkdir inter_data
mkdir advise_data

for i in {1..10}
do
  echo $i

  ./gen_inter $i inter_data/lane$i.txt
  python gen_inter_info_v3.py inter_data/lane$i.txt ../inter_info/lane_info$i
  ./gen_advise $i advise_data/advise$i.txt
  python gen_advise_info.py advise_data/advise$i.txt ../advise_info/advise_info$i
done
