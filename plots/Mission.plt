######## Plotscript-Header ########
reset
term = 'svg'
set terminal svg size 1280,960 enhanced font "Times, 12"
set output "plots/Mission.svg
set datafile separator ','
set key font "Times, 10" samplen 2
set autoscale
set xtic auto
set ytic auto

set style line 1 lt 2 lc rgb "blue" lw 1 pt 3 ps 0.25

######################################

set multiplot layout 5,3 columnsfirst scale 1,0.9

set xlabel "Horizontale Position"
set ylabel "Vert. Position"
plot 'Waypoints.csv' using 6:7 ls 1 notitle

set xlabel "Horizontale Position" 
set ylabel "Hor. Geschwindigkeit" 
plot "Waypoints.csv" using 6:4 ls 1 notitle

set xlabel "Vertikale Position" 
set ylabel "Vert. Geschwindigkeit" 
plot "Waypoints.csv" using 7:5 ls 1 notitle

set xlabel "Horizontale Position" 
set ylabel "Hor. Beschleunigung" 
plot "Waypoints.csv" using 6:2 ls 1 notitle

set xlabel "Vertikale Position" 
set ylabel "Vert. Beschleunigung" 
plot "Waypoints.csv" using 7:3 ls 1 notitle

set xlabel "Zeit" 
set ylabel "Hor. Position" 
plot "Waypoints.csv" using 1:6 ls 1 notitle

set xlabel "Zeit" 
set ylabel "Hor. Geschwindigkeit" 
plot "Waypoints.csv" using 1:4 ls 1 notitle

clear 

set xlabel "Zeit"
set ylabel "Hor. Beschleunigung"
plot "Waypoints.csv" using 1:2 ls 1 notitle

clear

set xlabel "Zeit"
set ylabel "Vert. Position"
plot "Waypoints.csv" using 1:7 ls 1 notitle

clear

set xlabel "Zeit" 
set ylabel "Vert. Geschwindigkeit"
plot "Waypoints.csv" using 1:5 ls 1 notitle

clear

set xlabel "Zeit"
set ylabel "Vert. Beschleunigung" 
plot "Waypoints.csv" using 1:3 ls 1 notitle

unset multiplot 