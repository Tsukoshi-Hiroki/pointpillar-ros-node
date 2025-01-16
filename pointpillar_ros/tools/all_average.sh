#!/bin/bash

#実行時は記入せよ 
input_dir="multi_3person_half_occulusion"

bash ./submodule/array_size_average.sh $input_dir 
sleep 3

bash ./submodule/array_size_average_average.sh $input_dir 
sleep 3

bash ./submodule/array_ped_size_average.sh $input_dir 
sleep 3

bash ./submodule/array_ped_size_average_average.sh $input_dir 
sleep 3

bash ./submodule/value_average.sh $input_dir 
sleep 3

bash ./submodule/value_average_average.sh $input_dir 
sleep 3

bash ./submodule/detection_ratio.sh $input_dir 
sleep 3
