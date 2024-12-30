
figure;tiledlayout(2,2)

% First plot
ax1 = nexttile; 
plot(T_arr,bias_est(:,1),'LineWidth',2);hold on;
plot(T_arr,bias_est(:,2),'LineWidth',2);hold on;
plot(T_arr,bias_est(:,3),'LineWidth',2);hold on;

ylabel('Bias [nT]')
legend("bx","by","bz")
grid minor

% Second plot
ax2 = nexttile; 
plot(T_arr,non_orth_est(:,1),'LineWidth',2);hold on;
plot(T_arr,non_orth_est(:,2),'LineWidth',2);hold on;
plot(T_arr,non_orth_est(:,3),'LineWidth',2);hold on;
plot(T_arr,non_orth_est(:,4),'LineWidth',2);hold on;
plot(T_arr,non_orth_est(:,5),'LineWidth',2);hold on;
plot(T_arr,non_orth_est(:,6),'LineWidth',2);hold on;

ylabel('Non Othogonality')
legend("d11","d22","d33","d12","d23","d13")
grid minor


% Third plot
ax3 = nexttile; 
plot(T_arr,mag_mes_ECEF(:,1),'LineWidth',2);hold on;
plot(T_arr,mag_mes_ECEF(:,2),'LineWidth',2);hold on;
plot(T_arr,mag_mes_ECEF(:,3),'LineWidth',2);hold on;

ylabel('B TRUE ECEF [nT]')
legend("BTRUEX","BTRUEY","BTRUEZ")
grid minor
% 4 plot
ax4 = nexttile; 
plot(T_arr,mag_mes_body(:,1),'LineWidth',2);hold on;
plot(T_arr,mag_mes_body(:,2),'LineWidth',2);hold on;
plot(T_arr,mag_mes_body(:,3),'LineWidth',2);hold on;

ylabel('B TAM BODY [nT]')
legend("BTAMX","BTAMY","BTAMZ")
grid minor

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7 ax8 ],'x')