clear
reset

DATA_0 = './0_kinematics-model.txt'
DATA_1 = './1_kinematics-model.txt'
DATA_2 = './2_kinematics-model.txt'
DATA_3 = './3_kinematics-model.txt'
DATA_4 = './4_kinematics-model.txt'
DATA_5 = './5_kinematics-model.txt'
DATA_6 = './6_kinematics-model.txt'
DATA_7 = './7_kinematics-model.txt'
DATA_8 = './8_kinematics-model.txt'
DATA_9 = './9_kinematics-model.txt'
DATA_10 = './10_kinematics-model.txt'
DATA_11 = './11_kinematics-model.txt'
DATA_12 = './12_kinematics-model.txt'

set size ratio -1
set size 0.8,0.8
set border lw 1.5

set xlabel 'x (m)' font 'Times,18'
set ylabel 'y (m)' font 'Times,18'
set xtics 1 font 'Times,18' nomirror
set ytics 1 font 'Times,18' nomirror

set xrange [-0.5:5]

set xzeroaxis
set yzeroaxis

#SOMA
# set style fill solid 0.0 border -1
# set style fill border lc "grey"
set arrow from 0.0,0.0 to 1.05,0.0 nohead lw 3 lc "black"
set arrow from 0.0,-0.5 to 0.0,0.5 nohead lw 3 lc "black"
set arrow from 1.05,-0.5 to 1.05,0.5 nohead lw 3 lc "black"

set object 1 rect from 0.0-0.45/2.0,0.9/2.0-0.2/2.0 to 0.0+0.45/2.0,0.9/2.0+0.2/2.0 fc "grey"
set object 2 rect from 0.0-0.45/2.0,-0.9/2.0-0.2/2.0 to 0.225,-0.9/2.0+0.2/2.0 fc "grey"
set object 3 rect from 1.05-0.45/2.0,0.35 to 1.05+0.45/2.0,0.55 fc "grey" 
set object 4 rect from 1.05-0.45/2.0,-0.55 to 1.05+0.45/2.0,-0.35 fc "grey"



set style data l
set linetype 1 lw 1.5 lc "black" ps 0.8 pt 6

plot DATA_0 u 1:2 lt 1 notitle, \
DATA_1 u 1:2 lt 1 notitle, \
DATA_2 u 1:2 lt 1 notitle, \
DATA_3 u 1:2 lt 1 notitle, \
DATA_4 u 1:2 lt 1 notitle, \
DATA_5 u 1:2 lt 1 notitle, \
DATA_6 u 1:2 lt 1 notitle, \
DATA_7 u 1:2 lt 1 notitle, \
DATA_8 u 1:2 lt 1 notitle, \
DATA_9 u 1:2 lt 1 notitle, \
DATA_10 u 1:2 lt 1 notitle, \
DATA_11 u 1:2 lt 1 notitle, \
DATA_12 u 1:2 lt 1 notitle

replot 'left-turn-actualcoord.txt' u 4:3 w p lt 1 notitle, \
'right-turn-actualcoord.txt' u 4:3 w p lt 1 notitle

set terminal postscript eps color enhanced
set output 'Turning-Radius.eps'
replot
