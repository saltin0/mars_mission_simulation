T_plot = T_arr(initial_index:end) - T_arr(initial_index);
bias_plot = bias_est(initial_index:end,:);
non_orth_plot = non_orth_est(initial_index:end,:);
bias_plot_true     = bias_true_arr(initial_index:end,:);
mes_est_angle_plot = angle_diff_MAG(initial_index:end);
est_true_angle_plot = angle_error_MAG(initial_index:end);
mes_true_angle_plot = angle_mes_error_MAG(initial_index:end);

%% BIAS Part
figure;tiledlayout(3,1)

% First plot
ax1 = nexttile; 
plot(T_plot,bias_plot_true(:,1),'LineWidth',2);hold on;
plot(T_plot,bias_plot(:,1),'LineWidth',2);hold on;
ylabel('Bias - x [nT]')
legend("True","Est")
grid minor

ax2 = nexttile; 
plot(T_plot,bias_plot_true(:,2),'LineWidth',2);hold on;
plot(T_plot,bias_plot(:,2),'LineWidth',2);hold on;
ylabel('Bias - y [nT]')
legend("True","Est")
grid minor

ax3 = nexttile; 
plot(T_plot,bias_plot_true(:,3),'LineWidth',2);hold on;
plot(T_plot,bias_plot(:,3),'LineWidth',2);hold on;
ylabel('Bias - z [nT]')
legend("True","Est")
grid minor

linkaxes([ax1 ax2 ax3],'x')


%% BIAS Errors
figure;tiledlayout(3,1)
% First plot
ax1 = nexttile; 
plot(T_plot,abs(bias_plot_true(:,1)-bias_plot(:,1)),'LineWidth',2);hold on;
ylabel('Bias Error - x [nT]')
grid minor

ax2 = nexttile; 
plot(T_plot,abs(bias_plot_true(:,2)-bias_plot(:,2)),'LineWidth',2);hold on;
ylabel('Bias Error - y [nT]')
grid minor

ax3 = nexttile; 
plot(T_plot,abs(bias_plot_true(:,3)- bias_plot(:,3)),'LineWidth',2);hold on;
ylabel('Bias Error - z [nT]')
grid minor

linkaxes([ax1 ax2 ax3],'x')


%% Non Orth Part

figure;tiledlayout(3,2)
% Third plot
ax1 = nexttile;
plot(T_plot,T_plot * 0 + 0.05,'LineWidth',2);hold on;
plot(T_plot,non_orth_plot(:,1),'LineWidth',2);hold on;
ylabel('Non Othogonality D11')
legend("True","Est")
grid minor

ax2 = nexttile;
plot(T_plot,T_plot * 0 + 0.1,'LineWidth',2);hold on;
plot(T_plot,non_orth_plot(:,2),'LineWidth',2);hold on;
ylabel('Non Othogonality D22')
legend("True","Est")
grid minor

ax3 = nexttile;
plot(T_plot,T_plot * 0 + 0.05,'LineWidth',2);hold on;
plot(T_plot,non_orth_plot(:,3),'LineWidth',2);hold on;
ylabel('Non Othogonality D33')
legend("True","Est")
grid minor

ax4 = nexttile;
plot(T_plot,T_plot * 0 + 0.05,'LineWidth',2);hold on;
plot(T_plot,non_orth_plot(:,4),'LineWidth',2);hold on;
ylabel('Non Othogonality D12')
legend("True","Est")
grid minor

ax5 = nexttile;
plot(T_plot,T_plot * 0 + 0.05,'LineWidth',2);hold on;
plot(T_plot,non_orth_plot(:,5),'LineWidth',2);hold on;
ylabel('Non Othogonality D13')
legend("True","Est")
grid minor

ax6 = nexttile;
plot(T_plot,T_plot * 0 + 0.05,'LineWidth',2);hold on;
plot(T_plot,non_orth_plot(:,6),'LineWidth',2);hold on;
ylabel('Non Othogonality D23')
legend("True","Est")
grid minor

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6  ],'x')
%% Non Orth Errors
figure;tiledlayout(3,2)
% Third plot
ax1 = nexttile;
plot(T_plot,abs(non_orth_plot(:,1)-0.05),'LineWidth',2);hold on;
ylabel('Non Orth Error D11')
legend("True","Est")
grid minor

ax2 = nexttile;
plot(T_plot,abs(non_orth_plot(:,2) - 0.1),'LineWidth',2);hold on;
ylabel('Non Orth Error D22')
legend("True","Est")
grid minor

ax3 = nexttile;
plot(T_plot,abs(non_orth_plot(:,3)-0.05),'LineWidth',2);hold on;
ylabel('Non Orth Error D33')
legend("True","Est")
grid minor

ax4 = nexttile;
plot(T_plot,abs(non_orth_plot(:,4)-0.05),'LineWidth',2);hold on;
ylabel('Non Orth Error D12')
legend("True","Est")
grid minor

ax5 = nexttile;
plot(T_plot,abs(non_orth_plot(:,5)-0.05),'LineWidth',2);hold on;
ylabel('Non Orth Error D13')
legend("True","Est")
grid minor

ax6 = nexttile;
plot(T_plot,abs(non_orth_plot(:,6)-0.05),'LineWidth',2);hold on;
ylabel('Non Orth Error D23')
legend("True","Est")
grid minor

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6  ],'x')


%% Angle Errors
figure;tiledlayout(3,1)

% First plot
ax1 = nexttile; 
plot(T_plot,mes_est_angle_plot,'LineWidth',2);hold on;
ylabel('Mes - Est Angle Diff [deg]')
grid minor

ax2 = nexttile; 
plot(T_plot,est_true_angle_plot,'LineWidth',2);hold on;
ylabel('True - Est Angle Diff [deg]')
grid minor

ax3 = nexttile; 
plot(T_plot,mes_true_angle_plot,'LineWidth',2);hold on;
ylabel('Mes - True Angle Diff [deg]')
grid minor

linkaxes([ax1 ax2 ax3],'x')



%% MAGNETOMETER MEASUREMENT VECTOR PLOT
figure;tiledlayout(2,1);
% First plot
ax1 = nexttile; 
plot(T_plot, mag_mes_body(initial_index:end,1));hold on;
plot(T_plot, mag_mes_body(initial_index:end,2));hold on;
plot(T_plot, mag_mes_body(initial_index:end,3));hold on;
ylabel('Body Frame Measurements [nT]')
legend("bx","by","bz")
grid minor

ax2 = nexttile; 
plot(T_plot,mag_mes_ECEF(initial_index:end,1));hold on;
plot(T_plot,mag_mes_ECEF(initial_index:end,2));hold on;
plot(T_plot,mag_mes_ECEF(initial_index:end,3));hold on;
legend("bx","by","bz")
ylabel('True Magnetometer Vector Inertial Frame [nT]')
grid minor


