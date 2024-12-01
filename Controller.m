classdef Controller
    properties
        flight_path_angle_rad
        gravity_turn_init_altitude_km
        controller_stage

    end
    
    methods
        function obj = Controller()
            obj.controller_stage = 0;
        
        end
        function [thrust_N, omega_cmd_rad,...
                  gamma_cmd_rad,gamma_dot_cmd_rad_s,...
                  alfa_cmd_rad,vel_cmd_m_s,delta_v_a_km_s,...
                  obj] = fp_vel_control(obj, ref_velocity_km_s     , ...
                                                      velocity_vector_a_km_s, ...
                                                      gamma_rad             , ...
                                                      aoa_rad               , ...
                                                      altitude_km           , ....
                                                      drag_N                , ...
                                                      gravity_force_N       , ...
                                                      full_thrust_N         , ...
                                                      mass_kg               , ...
                                                      earth_parking_orbit_alt_km, ...
                                                      position_vector_ecef_a_km)
            % Hyperparameter
            a_gamma        = 0.0089;
            a_vel          = 0.008;
            delta_v_a_km_s = [0.0,0.0,0.0];
           
            thrust_N = 0.0;
            zero_air_density_altittude_km = 20;
            gamma_0_rad = pi/2;

            alfa_cmd_rad = 0.0;
            omega_cmd_rad = 0.0;

            position_vector_direction_ecef = position_vector_ecef_a_km / norm(position_vector_ecef_a_km);
          
            vel_mag_km_s        = norm(velocity_vector_a_km_s);
            vel_cmd_m_s            = ref_velocity_km_s - ref_velocity_km_s * exp(-1 * a_vel * altitude_km);
%             accel_cmd_m_s2         = ref_velocity_km_s - ref_velocity_km_s * exp(-1 * a_vel * h_km);
            accel_cmd_m_s2      = (vel_cmd_m_s - vel_mag_km_s) * 20;
            thrust_N            = (accel_cmd_m_s2) * mass_kg + ((gravity_force_N * sin(gamma_rad) + drag_N * cos(aoa_rad)) / cos(aoa_rad));
            thrust_N            = min(max(0, thrust_N),full_thrust_N);

            if (altitude_km > zero_air_density_altittude_km)
                h_km = altitude_km - zero_air_density_altittude_km;
                gamma_cmd_rad          = gamma_0_rad * exp(-1 * a_gamma * h_km);
                gamma_dot_cmd_cl_rad_s = (gamma_cmd_rad - gamma_rad) * 1;
                gamma_dot_cmd_rad_s    = -1 * a_gamma * gamma_0_rad * exp(-1 * a_gamma * h_km) * vel_mag_km_s * sin(gamma_rad);
                gamma_dot_cmd_rad_s    =  gamma_dot_cmd_rad_s  + gamma_dot_cmd_cl_rad_s;

               
                if ((thrust_N < 1) ||(obj.controller_stage == 1))
                    alfa_cmd_rad = 0.0;
                    
                else
                    sin_         = (gravity_force_N * cos(gamma_rad) - vel_mag_km_s * 1000 * gamma_dot_cmd_rad_s * mass_kg) / thrust_N;
                    sin_         = min(max(sin_,-1.0),1.0);
                    alfa_cmd_rad = asin( sin_ );

                end
                omega_cmd_rad = (alfa_cmd_rad - aoa_rad) * 4;

            else
                gamma_cmd_rad       = gamma_0_rad;
                gamma_dot_cmd_rad_s = (gamma_cmd_rad - gamma_rad) * 1;
                omega_cmd_rad       = 0.0;
            end

            if (altitude_km >= earth_parking_orbit_alt_km)
                if (0 == obj.controller_stage)
                    obj.controller_stage = 1; % Now we are in parking orbit
                    parking_orbit_vel_direction = quatrotate([cosd(45),0.0,0.0,sind(45)],position_vector_direction_ecef);
                    parking_orbit_vel_a_km_s    = parking_orbit_vel_direction * ref_velocity_km_s;
                    delta_v_a_km_s              = parking_orbit_vel_a_km_s - velocity_vector_a_km_s;

                end
            end

            if (1 == obj.controller_stage)
                thrust_N = 0.0;
                omega_cmd_rad = 0.0;

            end



        end

    end
end

