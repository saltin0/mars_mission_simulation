%% Run Earth to Mars Simulation
clear;clc;close all;
% Load params
run("definitions.m");
addpath(genpath(cd));


%% Constants
m2km             = 1 / 1000;
total_sim_time_s = 4000;
sample_time_s    = 0.1;
total_sim_step   = total_sim_time_s / sample_time_s;


%% Init part
departure_location_ecef_km   = lla2ecef([0.0,0.0,0.0]) * m2km;
departure_velocity_ecef_km_s = [0.0,0.0,0.0];
q_ecef2b                     = [1.0, 0.0, 0.0, 0.0];
mass_kg                      = 730 * 1000; 

MS = MainSimulation(departure_location_ecef_km,departure_velocity_ecef_km_s,sample_time_s, earth_prm_st,q_ecef2b,mass_kg);


%% Data Holders
ecef_position_a_km = zeros(total_sim_step,3);
vel_a_km_s         = zeros(total_sim_step,1);
aoa_arr_deg        = zeros(total_sim_step,1);
fp_angle_arr_deg   = zeros(total_sim_step,1);


%% Simulation Loop
for i=1:total_sim_step
    MS = MS.simulate(9600000, [0.0,0.0,0.0]);

    ecef_position_a_km(i,:) = MS.spacecraft_pose_ecef_a_km;
    vel_a_km_s(i,1) = MS.calculate_vel();
    aoa_arr_deg(i,1) = MS.angle_of_attack_rad * 180 / pi;
    fp_angle_arr_deg(i,1) = MS.flight_path_angle_rad * 180 / pi;
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
plot3(ecef_position_a_km(:,1),ecef_position_a_km(:,2),ecef_position_a_km(:,3),'LineWidth',2)

figure;
subplot('221')
plot(vel_a_km_s)
ylabel('Vel')

subplot('222')
plot(aoa_arr_deg)
ylabel('alpha')

subplot('223')
plot(fp_angle_arr_deg)
ylabel('gamma')
