#!/bin/bash

start=2
end=10
increment=2
d_sense=15
# v_0=8
a_b=5
t_react=0.00

for ((x=start; x<=end; x+=increment))
do
    printf "Running with d_sense=${d_sense} v_0=${x} a_b=${a_b} t_react=${t_react}\n"
    python3 set_pos.py --x 0.0 --y 0.0 && python3 main.py --d_sense $d_sense --v_0 $x --a_b $a_b --t_react $t_react
    
    # t_react=$(echo "scale=2; $x/$end" | bc -l)
    # printf "Running with d_sense=${d_sense} v_0=${v_0} a_b=${a_b} t_react=${t_react}\n"
    # python3 set_pos.py --x 0.0 --y 0.0 && python3 main.py --d_sense $d_sense --v_0 $v_0 --a_b $a_b --t_react $t_react
    
    printf "Finished\n"
done

