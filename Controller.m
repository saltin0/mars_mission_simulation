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
                  alfa_cmd_rad,obj] = fp_vel_control(obj, ref_velocity_km_s     , ...
                                                      velocity_vector_a_km_s, ...
                                                      gamma_rad             , ...
                                                      aoa_rad               , ...
                                                      altitude_km           , ....
                                                      drag_N                , ...
                                                      gravity_force_N       , ...
                                                      full_thrust_N         , ...
                                                      mass_kg               )
            % Hyperparameter
            a = 0.0089*0.5;
           
            thrust_N = 0.0;
            zero_air_density_altittude_km = 20;
            gamma_0_rad = pi/2;

            alfa_cmd_rad = 0.0;
            omega_cmd_rad = 0.0;
          
            vel_mag_km_s        = norm(velocity_vector_a_km_s);

            if (obj.controller_stage == 0)
                if ((ref_velocity_km_s - vel_mag_km_s) > 0)
                    thrust_N = full_thrust_N;
                end

                if(abs(ref_velocity_km_s - vel_mag_km_s) < 0.0001)
                    obj.controller_stage = 1;
                end
        
            elseif (obj.controller_stage == 1)
                thrust_N = (gravity_force_N * sin(gamma_rad) + drag_N * cos(aoa_rad)) / cos(aoa_rad); 
            end

            if (altitude_km > zero_air_density_altittude_km)
                h_km = altitude_km - zero_air_density_altittude_km;
                gamma_cmd_rad          = gamma_0_rad * exp(-1 * a * h_km);
                gamma_dot_cmd_cl_rad_s = (gamma_cmd_rad - gamma_rad) * 1;
                gamma_dot_cmd_rad_s    = -1 * a * gamma_0_rad * exp(-1 * a * h_km) * vel_mag_km_s * sin(gamma_rad);
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

        end

    end
end

