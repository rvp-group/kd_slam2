set terminal pdfcairo enhanced color
set xlabel "time [s]"
set title  filetitle
set output ate_r_plot
plot err_filename using  1:2 w l title 'ate-r[deg]'

set output ate_t_plot
plot err_filename using  1:3 w l title 'ate-t[m]'

set output rpe_r_plot
plot err_filename using  1:4 w l title 'rpe-r[deg]'

set output rpe_t_plot
plot err_filename using  1:5 w l title 'rpe-t[m]'

unset xlabel
set output traj_xy_plot
set size ratio -1
plot gt_traj_file using  2:3 w l title 'gt', ev_traj_file using 2:3 w l title 'es'

set output traj_yz_plot
set size ratio -1
plot gt_traj_file using  3:4 w l title 'gt', ev_traj_file using 3:4 w l title 'es'

set output traj_xz_plot
set size ratio -1
plot gt_traj_file using  2:4 w l title 'gt', ev_traj_file using 2:4 w l title 'es'

