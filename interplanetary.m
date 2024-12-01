function Delta_v = interplanetary(phase_angle_rad,...
    mars_w_rad_s, ...
    earth_w_rad_s,...
    mars_orbit_inclination_rad, ...
    r_randezvous_km,...
    mu_sun_km3_s2,...
    ra_km)
%% Non Hohman Interplanetary Transfer Calculation
mars_w_helio_rad_s = mars_w_rad_s * cos(mars_orbit_inclination_rad);

% All depends on randezvous time
theta_rand_rad = deg2rad(160);
time_rand = 100000;
theta_rand_rad = time_rand * mars_w_helio_rad_s + phase_angle_rad;
transfer_orbit_period_s     = (2*pi)  / theta_rand_rad * time_rand;

transfer_orbit_semi_major_m = ((mu_sun_km3_s2 * transfer_orbit_period_s^2) / (4*pi^2))^(1/3);
rp_km = 2*transfer_orbit_semi_major_m - ra_km;
e = (ra_km - rp_km) / (ra_km + rp_km);




end