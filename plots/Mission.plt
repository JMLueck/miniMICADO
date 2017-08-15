######## Plotscript-Header ########
reset
term = 'svg'
#set terminal svg size 1280,960 enhanced font "Times, 12"
set terminal postscript eps size 75 cm, 56 cm colour enhanced "Times-Roman, 96"
set output "plots/Mission.eps"
set datafile separator ';'
set key font "Times, 10" samplen 2
set autoscale
set xtic auto
set ytic auto

set style line 1 lt 2 lc rgb "blue" lw 1 pt 3 ps 0.25

######################################

set multiplot layout 3,1 columnsfirst scale 1,0.9

set xlabel "Horizontal Position"
set ylabel "Vertical Position"
set xrange [-5:1600]
set yrange [-5:120]
plot 'Waypoints.csv' using 6:7 ls 1 notitle

set xlabel "Time" 
set ylabel "Horizontal Velocity" 
set xrange [-5:225]
set yrange [-5:30]
plot "Waypoints.csv" using 1:4 ls 1 notitle

set xlabel "Time" 
set ylabel "Vertical Velocity" 
set xrange [-5:225]
set yrange [-5:5]
plot "Waypoints.csv" using 1:5 ls 1 notitle

unset multiplot 