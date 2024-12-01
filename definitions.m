%% This file contains parameters about the celestial objects and orbits
%% Time Calculations
t = datetime(2035, 6, 1, 0, 0, 0); % Belirli tarih ve saat

% Dünya'nın konumu (Güneş'e göre)
au = 149597871;
earth_pos = planetEphemeris(juliandate(t), 'SolarSystem', 'Earth') / au;

% Mars'ın konumu (Güneş'e göre)
mars_pos = planetEphemeris(juliandate(t), 'SolarSystem', 'Mars') / au;

% Sonuçları göster (km cinsinden)
fprintf('Dünya Konumu: [%.2f, %.2f, %.2f] km\n', earth_pos);
fprintf('Mars Konumu: [%.2f, %.2f, %.2f] km\n', mars_pos);

e = earth_pos(1:2);

m = mars_pos(1:2);

me_dot = dot(m,e);
phase_angle_deg = acosd(me_dot/ (norm(e) * norm(m)));
%% 
earth_prm_st              = struct;
earh_parking_orbit_prm_st = struct;
% ballistic_orbit_prm_st    = struct;
%% Struct definitions
sun_prm_st.mu_km3_s2    = 1.32712e11; 

earth_prm_st.radius_km  = 6378  ;   % [km]
earth_prm_st.mu_km3_s2  = 398600;   % [km^3/s^2]
earth_prm_st.a_km       = 149.6e6; % Semimajor axis of earth
v_earth_km_s            = sqrt(sun_prm_st.mu_km3_s2 / earth_prm_st.a_km);
w_earth_deg_s           = (360) / ((2 * pi * earth_prm_st.a_km) / v_earth_km_s)


mars_prm_st.mass_kg     = 641.9e21;    % [kg]
mars_prm_st.radius_m    = 3396  ;    % [m]
mars_prm_st.a_km        = 227.956e6; % [km]
mars_prm_st.inclination_rad = 1.845 * pi / 180; 
v_mars_km_s             = sqrt(sun_prm_st.mu_km3_s2 / mars_prm_st.a_km);
w_mars_deg_s            = (360) / ((2 * pi * mars_prm_st.a_km) / v_mars_km_s);


t_catch_s = 365.2296 / abs(w_earth_deg_s - w_mars_deg_s);
t_catch_days = t_catch_s / (24 * 3600)


% Earth parking orbit parameters - Circular orbits
earh_parking_orbit_prm_st.radius_km     = earth_prm_st.radius_km + 500; % [km] --> Design criteria
orbit_velocity_km_s = sqrt(earth_prm_st.mu_km3_s2 / earh_parking_orbit_prm_st.radius_km); 
earh_parking_orbit_prm_st.velocity_km_s = orbit_velocity_km_s ;

v_transfer_km_s = 33.8154;
v_inf           = v_transfer_km_s - v_earth_km_s;
v_park = earh_parking_orbit_prm_st.velocity_km_s;
delta_v         =  v_park * (sqrt(2 + (v_inf/ v_park)^2) -1);

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


% ballistic_orbit_prm_st.eccentricity = e;


