%% This file contains parameters about the celestial objects and orbits
earth_prm_st              = struct;
earh_parking_orbit_prm_st = struct;
% ballistic_orbit_prm_st    = struct;
%% Struct definitions
earth_prm_st.radius_km  = 6378  ; % [km]
earth_prm_st.mu_km3_s2  = 398600; % [km^3/s^2]

% Earth parking orbit parameters - Circular orbits
earh_parking_orbit_prm_st.radius_km     = earth_prm_st.radius_km + 500; % [km] --> Design criteria
orbit_velocity_km_s = sqrt(earth_prm_st.mu_km3_s2 / earh_parking_orbit_prm_st.radius_km); 
earh_parking_orbit_prm_st.velocity_km_s = orbit_velocity_km_s ;

% Earth to parking orbit ballistic trajectory parameters
% ballistic_orbit_prm_st.apogee_length_km       = earh_parking_orbit_prm_st.radius_km    ; 
% ballistic_orbit_prm_st.apogee_velocity_km_s   = earh_parking_orbit_prm_st.velocity_km_s * 0.8;
% ballistic_orbit_prm_st.angular_momentum_km2_s = ballistic_orbit_prm_st.apogee_length_km * ...
%                                                 ballistic_orbit_prm_st.apogee_velocity_km_s;
% 
% h  = ballistic_orbit_prm_st.angular_momentum_km2_s;
% mu = earth_prm_st.mu_km3_s2;
% ra = ballistic_orbit_prm_st.apogee_length_km;
% true_anomaly_apogee_deg = 180; % [deg]
% e   = ((h^2 / (mu * ra)) - 1) / cosd(true_anomaly_apogee_deg); 


ballistic_orbit_prm_st.eccentricity = e;


