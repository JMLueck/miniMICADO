set terminal png # gnuplot recommends setting terminal before output

set title "Sizing Plot"
set xlabel "W/S"
set ylabel "T/W"
set grid
set datafile separator ";"

set style line 1 lt 1 lc rgb "red" lw 2
set style line 2 lt 1 lc rgb "orange" lw 2
set style line 3 lt 1 lc rgb "yellow" lw 2
set style line 4 lt 1 lc rgb "green" lw 2
set style line 5 lt 1 lc rgb "blue" lw 4

plot 	"initialSizing_plot.csv" using 1:2 ls 1 with lines title "Landing",\
	"initialSizing_plot.csv" using 3:4 ls 2 with lines title "Climb",\
	"initialSizing_plot.csv" using 5:6 ls 3 with lines title "Takeoff",\
	"initialSizing_plot.csv" using 7:8 ls 4 with lines title "Cruise",\
	"initialSizing_plot.csv" using 9:10 ls 5 with points title "Result"
	
set style line 6 lt 2 lw 3
set key box ls 6

#replot

