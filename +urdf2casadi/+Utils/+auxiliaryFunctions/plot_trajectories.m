function ret = plot_joint_trajectories(jointTraj, timespan, name_var)
  figure;
  p = plot(timespan, jointTraj);
  set(p, "linewidth", 1);
  set(gca, "linewidth", 3, "fontsize", 15);
  %l = legend('Joint 1','Joint 2','Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
  title(name_var)
  xlabel('time','fontsize',25)
  ylabel(name_var,'fontsize',25)
end