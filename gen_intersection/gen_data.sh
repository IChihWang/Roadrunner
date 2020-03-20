#!/bin/bash

mkdir advise_info
mkdir inter_info
cd gen_intersection


echo $1
./gen_inter $1 inter_data/lane$1.txt
python gen_inter_info_v3.py inter_data/lane$1.txt lane_info$1

./gen_advise $1 advise_data/advise$1.txt
python gen_advise_info.py advise_data/advise$1.txt advise_info$1
