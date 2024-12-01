
figure;tiledlayout(3,3)

% First plot
ax1 = nexttile; 
plot(T_arr,vel_a_km_s,'LineWidth',2);hold on;
plot(T_arr,vel_cmd_arr_km_s,'LineWidth',2);
ylabel('Vel')
legend("Est","Cmd")
grid minor

% Second plot
ax2 = nexttile;
plot(T_arr,aoa_arr_deg,'LineWidth',2);hold on;
plot(T_arr,alpha_cmd_deg,'LineWidth',2)
ylabel('alpha')
legend(["Est","Cmd"])
grid minor
% Third plot
ax3 = nexttile;
plot(T_arr,fp_angle_arr_deg,'LineWidth',2);hold on;
plot(T_arr,gamma_cmd_arr_deg,'LineWidth',2)
ylabel('gamma')
legend(["Est","Cmd"])
grid minor
% 4 plot
ax4 = nexttile;
plot(T_arr,gamma_dot_cmd_deg_s,'LineWidth',2)
ylabel('gamma dot')
legend(['Cmd'])
grid minor
% 5 plot
ax5 = nexttile;
plot(T_arr,omega_arr_deg_s,'LineWidth',2)
ylabel('omega')
legend(['Cmd'])
grid minor
% 6 plot
ax6 = nexttile;
plot(T_arr,pitch_arr_deg,'LineWidth',2)
ylabel('pitch')
legend(['Est'])
grid minor
% 7 plot
ax7 = nexttile;
plot(T_arr,thrust_arr_N,'LineWidth',2)
ylabel('Thrust')
legend(['Cmd'])
grid minor
% 7 plot
ax8 = nexttile;
plot(T_arr,altitude_arr_km,'LineWidth',2)
ylabel('Altitude')
legend(['Est'])
grid minor


linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7 ax8 ],'x')