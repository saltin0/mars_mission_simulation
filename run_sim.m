%% Run Earth to Mars Simulation
clear;clc;close all;
% Load params
run("definitions.m");
addpath(genpath(cd));


%% Constants
m2km             = 1 / 1000;
total_sim_time_s = 500*0 + 1 * 10000;
sample_time_s    = 0.05;
total_sim_step   = total_sim_time_s / sample_time_s;

%% 
max_thrust_N = 9600000;
C = Controller();
thrust_N    = 0.0;
omega_rad_s = 0.0;


%% Init part
departure_location_ecef_km   = lla2ecef([0.0,0.0,500.0]) * m2km* 1 + 0 *[earth_prm_st.radius_km + 500,0,0];
departure_velocity_ecef_km_s = [1/ 1000,0.0,0.0]*1 + 0*[0.0,earh_parking_orbit_prm_st.velocity_km_s,0.0] ;
departure_accel_ecef_km_s2   = [0.0,0.0,0.0];
q_ecef2b                     = [1.0, 0.0, 0.0, 0.0];
mass_kg                      = 730 * 1000; 

MS = MainSimulation(departure_location_ecef_km,departure_velocity_ecef_km_s,departure_accel_ecef_km_s2,sample_time_s, earth_prm_st,q_ecef2b,mass_kg);


%% Data Holders
ecef_position_a_km  = zeros(total_sim_step,3);
vel_a_km_s          = zeros(total_sim_step,1);
aoa_arr_deg         = zeros(total_sim_step,1);
fp_angle_arr_deg    = zeros(total_sim_step,1);
gamma_cmd_arr_deg   = zeros(total_sim_step,1);
gamma_dot_cmd_deg_s = zeros(total_sim_step,1);
alpha_cmd_deg       = zeros(total_sim_step,1);
omega_arr_deg_s     = zeros(total_sim_step,1);
pitch_arr_deg       = zeros(total_sim_step,1);
thrust_arr_N        = zeros(total_sim_step,1);
altitude_arr_km     = zeros(total_sim_step,1);

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
     alfa_cmd_rad,C] = C.fp_vel_control(ref_velocity_km_s     , ...
                                      velocity_vector_a_km_s, ...
                                      gamma_rad             , ...
                                      aoa_rad               , ...
                                      altitude_km           , ....
                                      0.0                   , ...
                                      gravity_force_N       , ...
                                      max_thrust_N          , ...
                                      MS.spacecraft_mass_kg);

    MS = MS.simulate(thrust_N , [0.0,0.0,omega_rad_s]);



    ecef_position_a_km(i,:) = MS.spacecraft_pose_ecef_a_km;
    vel_a_km_s(i,1) = MS.calculate_vel();
    aoa_arr_deg(i,1) = MS.angle_of_attack_rad * 180 / pi;
    fp_angle_arr_deg(i,1) = MS.flight_path_angle_rad * 180 / pi;
    gamma_cmd_arr_deg(i,1) = gamma_cmd_rad * 180 / pi;
    gamma_dot_cmd_deg_s(i,1) = gamma_dot_cmd_rad_s * 180 / pi;
    alpha_cmd_deg(i,1) = alfa_cmd_rad * 180 / pi;
    omega_arr_deg_s(i,1) = omega_rad_s * 180 / pi;
    euler = quat2eul(MS.q_ecef2b);
    pitch_arr_deg(i,1)   = euler(1);
    thrust_arr_N(i,1)    = thrust_N;
    altitude_arr_km(i,1) = MS.altitude_km;
end

%% Plot 
% % Dünya haritasını yükle
% earthImage = imread('earthimage.jpg'); 
% 
% [lat, lon] = meshgrid(-90:1:90, -180:1:180); 
% [xe, ye, ze] = sph2cart(deg2rad(lon), deg2rad(lat), earth_prm_st.radius_km); 
% 
% figure;
% surface(xe, ye, ze, flipud(earthImage), 'FaceColor', 'texturemap', 'EdgeColor', 'none');
% axis equal;
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title('LEO');
% hold on;
figure; hold on;
azz = 0:0.01:2*pi;
plot3(ecef_position_a_km(:,1),ecef_position_a_km(:,2),ecef_position_a_km(:,3),'LineWidth',2,'Color','r')
plot3(earth_prm_st.radius_km * cos(azz),earth_prm_st.radius_km * sin(azz),(earth_prm_st.radius_km * sin(azz)*0),'LineWidth',2,'Color','b')

pilot_graphs;

lla_arr = ecef2lla(ecef_position_a_km * 1000);
figure;
subplot('311')
plot(lla_arr(:,1))
ylabel('Lat')

subplot('312')
plot(lla_arr(:,2))
ylabel('Lon')

subplot('313')
plot(lla_arr(:,3))
ylabel('Alt')


