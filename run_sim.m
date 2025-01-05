%% Run Earth to Mars Simulation
clear;clc;close all;
% Load params
run("definitions.m");
addpath(genpath(cd));


%% Constants
m2km             = 1 / 1000;
total_sim_time_s = 500*0 + 1 * 8000;
sample_time_s    = 0.05 *2;
total_sim_step   = total_sim_time_s / sample_time_s;


%% 
max_thrust_N = 9600000;
C = Controller();
thrust_N    = 0.0;
omega_rad_s = 0.0;

%% Initial Estirmator Parameters
C.b_MAG = [5000,3000,6000]';
C.D_MAG = [0.05,0.1,0.05,0.05,0.05,0.05]'*1;
C.P_EKF = [1000*eye(3) zeros(3,6);
           zeros(6,3)  0.001*eye(6)];

%% Init part
departure_location_ecef_km   = lla2ecef([0.0,0.0,500.0]) * m2km* 0 + 1 *[earth_prm_st.radius_km,0,0];
departure_velocity_ecef_km_s = [1/ 1000,0.0,0.0]*1 + 0*[0.0,earh_parking_orbit_prm_st.velocity_km_s,0.0] ;
departure_accel_ecef_km_s2   = [0.0,0.0,0.0];
q_ecef2b                     = [1.0, 0.0, 0.0, 0.0];
mass_kg                      = 730 * 1000; 
parking_orbit_altitude_km    = earh_parking_orbit_prm_st.radius_km - earth_prm_st.radius_km;
MS = MainSimulation(departure_location_ecef_km,departure_velocity_ecef_km_s,departure_accel_ecef_km_s2,sample_time_s, earth_prm_st,q_ecef2b,mass_kg);

bias_type = 0; % 0 : Constant, 1 : Not Constant

%% Data Holders
zero_arr            = zeros(total_sim_step,1);
ecef_position_a_km  = zeros(total_sim_step,3);
vel_a_km_s          = zero_arr;
aoa_arr_deg         = zero_arr;
fp_angle_arr_deg    = zero_arr;
gamma_cmd_arr_deg   = zero_arr;
gamma_dot_cmd_deg_s = zero_arr;
alpha_cmd_deg       = zero_arr;
omega_arr_deg_s     = zeros(total_sim_step,3);
pitch_arr_deg       = zero_arr;
thrust_arr_N        = zero_arr;
altitude_arr_km     = zero_arr;
vel_cmd_arr_km_s    = zero_arr;
T_arr               = zero_arr;
mag_mes_ECEF        = ecef_position_a_km;
mag_mes_body        = ecef_position_a_km;
bias_est            = ecef_position_a_km;
non_orth_est        = zeros(total_sim_step,6);
angle_diff_MAG      = zero_arr;
angle_error_MAG     = zero_arr;
angle_mes_error_MAG = zero_arr;
bias_true_arr            = zeros(total_sim_step,3);
B_true_ECEF_est_inp = [0;0;0];
B_mes_BODY_est_inp  = [0;0;0];
N_update            = round(1 / sample_time_s);
initial_index       = 1;
mag_mes_ECEF_bs     = ecef_position_a_km;

%% Simulation Loop
for i=1:total_sim_step
    altitude_km            = MS.altitude_km;
    aoa_rad                = MS.angle_of_attack_rad;
    gamma_rad              = MS.flight_path_angle_rad;
    velocity_vector_a_km_s = MS.spacecraft_vel_ecef_a_km_s;
    ref_velocity_km_s      = earh_parking_orbit_prm_st.velocity_km_s;
    gravity_force_N        = MS.gravity_force_N; 

    [thrust_N,omega_rad_s,...
     gamma_cmd_rad,gamma_dot_cmd_rad_s,...
     alfa_cmd_rad,vel_cmd_m_s,delta_v_a_km_s,...
     reset_orientation,C] = C.fp_vel_control(ref_velocity_km_s     , ...
                                             velocity_vector_a_km_s, ...
                                             gamma_rad             , ...
                                             aoa_rad               , ...
                                             altitude_km           , ....
                                             0.0                   , ...
                                             gravity_force_N       , ...
                                             max_thrust_N          , ...
                                             MS.spacecraft_mass_kg ,...
                                             parking_orbit_altitude_km, ...
                                             MS.spacecraft_pose_ecef_a_km);

    MS = MS.simulate(thrust_N , omega_rad_s,delta_v_a_km_s, reset_orientation);

 
    [MS, B_true_BODY, B_mes_BODY, B_true_ECEF, B_mes_ECEF] = MS.magnetometer_model(bias_type);


    if (0 == mod(i,N_update))
        B_true_ECEF_est_inp = B_true_ECEF;
        B_mes_BODY_est_inp  = B_mes_BODY;
    end

    [B_cor_BODY,C] = C.estimator(B_true_ECEF_est_inp, B_mes_BODY_est_inp);

    if (true == reset_orientation)
        initial_index = i;
    end


    dot_product = min(dot(B_cor_BODY/norm(B_cor_BODY),B_mes_BODY/norm(B_mes_BODY)),1.0);
    angle_diff_MAG(i) = acosd(dot_product);
    
    dot_product = min(dot(B_cor_BODY/norm(B_cor_BODY),B_true_BODY/norm(B_true_BODY)),1.0);
    angle_error_MAG(i) = acosd(dot_product);

    dot_product = min(dot(B_mes_BODY/norm(B_mes_BODY),B_true_BODY/norm(B_true_BODY)),1.0);
    angle_mes_error_MAG(i) = acosd(dot_product);

    bias_true_arr(i,:)      = reshape(MS.bias_vector_nT, [1,3]);
    ecef_position_a_km(i,:) = MS.spacecraft_pose_ecef_a_km;
    vel_a_km_s(i,1) = MS.calculate_vel();
    aoa_arr_deg(i,1) = MS.angle_of_attack_rad * 180 / pi;
    fp_angle_arr_deg(i,1) = MS.flight_path_angle_rad * 180 / pi;
    gamma_cmd_arr_deg(i,1) = gamma_cmd_rad * 180 / pi;
    gamma_dot_cmd_deg_s(i,1) = gamma_dot_cmd_rad_s * 180 / pi;
    alpha_cmd_deg(i,1) = alfa_cmd_rad * 180 / pi;
    omega_arr_deg_s(i,:) = omega_rad_s * 180 / pi;
    euler = quat2eul(MS.q_ecef2b);
    pitch_arr_deg(i,1)   = euler(1);
    thrust_arr_N(i,1)    = thrust_N;
    altitude_arr_km(i,1) = MS.altitude_km;
    vel_cmd_arr_km_s(i,1) = vel_cmd_m_s;
    T_arr(i,1)            = MS.time_s;
    mag_mes_ECEF(i,:)     = B_true_ECEF';
    mag_mes_body(i,:)     = B_mes_BODY';
    bias_est    (i,:)     = reshape(C.b_MAG,[1,3]);
    non_orth_est(i,:)     = reshape(C.D_MAG,[1,6]);
    mag_mes_ECEF_bs(i,:)  = reshape(B_mes_ECEF,[1,3]);
end

%% Plot States

figure; hold on;
azz = 0:0.01:2*pi;
plot3(ecef_position_a_km(:,1),ecef_position_a_km(:,2),ecef_position_a_km(:,3),'LineWidth',2,'Color','r')
plot3(earth_prm_st.radius_km * cos(azz),earth_prm_st.radius_km * sin(azz),(earth_prm_st.radius_km * sin(azz)*0),'LineWidth',2,'Color','b')
grid minor
xlabel("ECEF - x")
ylabel("ECEF - y")
pilot_graphs;


%% Visualize Orbit
figure;
plot3(ecef_position_a_km(:,1),ecef_position_a_km(:,2),ecef_position_a_km(:,3),'LineWidth',2,'Color','r')
hold on;
plot_arr = 1:6000:size(ecef_position_a_km,1);
mag_mes_ECEF_bs_ = mag_mes_ECEF_bs / 10;
quiver3(ecef_position_a_km(plot_arr,1),ecef_position_a_km(plot_arr,2),ecef_position_a_km(plot_arr,3), ...
    mag_mes_ECEF_bs_(plot_arr,1), mag_mes_ECEF_bs_(plot_arr,2), mag_mes_ECEF_bs_(plot_arr,3), ...
            0, 'LineWidth', 1, 'MaxHeadSize', 0.5); % 3D ok çizimi

hold on;

% Earth Plot
[earth_x, earth_y, earth_z] = sphere(50);
surf(earth_x * 6371, earth_y * 6371, earth_z * 6371, 'FaceColor', 'c', 'EdgeColor', 'none', 'FaceAlpha', 0.5);
axis equal;
grid on;
xlabel('X (km)');
ylabel('Y (km)');
zlabel('Z (km)');
title('Celestial Object in Parking Orbit');
legend('Orbit', 'Magnetometer Measurements','Earth');

%% Plot Magnetometer
plot_magnetometer;


