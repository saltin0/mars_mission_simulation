
figure;tiledlayout(3,3)

% First plot
ax1 = nexttile;
plot(vel_a_km_s)
ylabel('Vel')

% Second plot
ax2 = nexttile;
plot(aoa_arr_deg);hold on;
plot(alpha_cmd_deg)
ylabel('alpha')
legend(["Est","Cmd"])

% Third plot
ax3 = nexttile;
plot(fp_angle_arr_deg);hold on;
plot(gamma_cmd_arr_deg)
ylabel('gamma')
legend(["Est","Cmd"])

% 4 plot
ax4 = nexttile;
plot(gamma_dot_cmd_deg_s)
ylabel('gamma dot')
legend(['Cmd'])

% 5 plot
ax5 = nexttile;
plot(omega_arr_deg_s)
ylabel('omega')
legend(['Cmd'])

% 6 plot
ax6 = nexttile;
plot(pitch_arr_deg)
ylabel('pitch')
legend(['Cmd'])

% 7 plot
ax7 = nexttile;
plot(thrust_arr_N)
ylabel('Thrust')
legend(['Cmd'])

% 7 plot
ax8 = nexttile;
plot(altitude_arr_km)
ylabel('Altitude')
legend(['Est'])



linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7 ax8 ],'x')