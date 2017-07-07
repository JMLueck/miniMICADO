# ID$ : girl.gp
# Robert Billon, 2001         
#
# Modified from the HP 9845 Basic by G. Wesley 12/29/82
# Translated for Qbasic by Robert Billon, 1994-01-01
# Adaped for Gnuplot by Robert Billon, 2001-11-18
# Needs girl.dat datafile
# 
set title "DEMO GNUPLOT LINUX"
set label 1 "Gnuplot peut tracer une courbe d'après un fichier de données" at 75,244
set label 2 "Gnuplot can plot a curve from a datafile" at 115,234
set nokey
set xrange [0:400]
set yrange [0:240]
plot "girl.dat" with lines 8
pause 15
reset

