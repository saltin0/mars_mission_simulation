classdef Controller
    % Estimator Ref : 
    properties
        flight_path_angle_rad
        gravity_turn_init_altitude_km
        controller_stage
        P_EKF
        b_MAG
        D_MAG
        estimator_run_cnt

    end
    
    methods
        function obj = Controller()
            obj.controller_stage = 0;
            obj.estimator_run_cnt = 0;
        
        end
        function [thrust_N, omega_cmd_rad,...
                  gamma_cmd_rad,gamma_dot_cmd_rad_s,...
                  alfa_cmd_rad,vel_cmd_m_s,delta_v_a_km_s,...
                  reset_orientation,obj] = fp_vel_control(obj, ref_velocity_km_s     , ...
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

            reset_orientation = false;

            alfa_cmd_rad = 0.0;
            omega_cmd_rad = [0.0,0.0,0.0];

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
                omega_cmd_rad(3) = (alfa_cmd_rad - aoa_rad) * 4;

            else
                gamma_cmd_rad       = gamma_0_rad;
                gamma_dot_cmd_rad_s = (gamma_cmd_rad - gamma_rad) * 1;
                omega_cmd_rad(3)       = 0.0;
            end

            if (altitude_km >= earth_parking_orbit_alt_km)
                if (0 == obj.controller_stage)
                    obj.controller_stage = 1; % Now we are in parking orbit
                    reset_orientation = true;
                    parking_orbit_vel_direction = quatrotate([cosd(-45),0.0,0.0,sind(-45)],position_vector_direction_ecef);
                    parking_orbit_vel_a_km_s    = parking_orbit_vel_direction * ref_velocity_km_s;
                    delta_v_a_km_s              = (parking_orbit_vel_a_km_s - velocity_vector_a_km_s);

                end
            end

            if (1 == obj.controller_stage)
                reset_orientation = true;
                % Tumbling motion initiated 
                thrust_N = 0.0;
                omega_cmd_rad = deg2rad([-0.1,0.5,0.5]) * 0.0;
                if (100 == obj.estimator_run_cnt)
                    obj.controller_stage = 2;
                    thrust_N = 0.0;
                end
                obj.estimator_run_cnt = obj.estimator_run_cnt + 1;
                

            end

            if (2 == obj.controller_stage)
                omega_cmd_rad = deg2rad([-0.1,0.5,0.5]);
                thrust_N = 0.0;
            end



        end

        function [B_corrected,obj] = estimator(obj,B_true_ECEF, B_mes_BODY)
            B_corrected = B_mes_BODY;
            if (2 == obj.controller_stage)
                S = [B_mes_BODY(1)^2, B_mes_BODY(2)^2, ...
                     B_mes_BODY(3)^2, 2*B_mes_BODY(1)*B_mes_BODY(2),...
                     2*B_mes_BODY(1)*B_mes_BODY(3), 2*B_mes_BODY(2)*B_mes_BODY(3)];
            
                D = [obj.D_MAG(1) obj.D_MAG(4) obj.D_MAG(5);...
                     obj.D_MAG(4) obj.D_MAG(2) obj.D_MAG(6);...
                     obj.D_MAG(5) obj.D_MAG(6) obj.D_MAG(3)];
    
                J = [B_mes_BODY(1)*obj.b_MAG(1), ...
                     B_mes_BODY(2)*obj.b_MAG(2), ...
                     B_mes_BODY(3)*obj.b_MAG(3), ...
                     B_mes_BODY(1)*obj.b_MAG(2)+B_mes_BODY(2)*obj.b_MAG(1), ...
                     B_mes_BODY(1)*obj.b_MAG(3)+B_mes_BODY(3)*obj.b_MAG(1), ...
                     B_mes_BODY(2)*obj.b_MAG(3)+B_mes_BODY(3)*obj.b_MAG(2)];
    
               dEdD = [2*(1+obj.D_MAG(1)), 0, 0, 2*obj.D_MAG(4), 2*obj.D_MAG(5), 0;
                       0, 2*(1+obj.D_MAG(2)), 0, 2*obj.D_MAG(4), 0, 2*obj.D_MAG(6);
                       0, 0, 2*(1+obj.D_MAG(3)), 0, 2*obj.D_MAG(5), 2*obj.D_MAG(6);
                       obj.D_MAG(4), obj.D_MAG(4), 0 2+obj.D_MAG(1)+obj.D_MAG(2), obj.D_MAG(6), obj.D_MAG(5);
                       obj.D_MAG(5), 0, obj.D_MAG(5), obj.D_MAG(6), 2+obj.D_MAG(1)+obj.D_MAG(3), obj.D_MAG(4);
                       0, obj.D_MAG(6), obj.D_MAG(6), obj.D_MAG(5), obj.D_MAG(4), 2+obj.D_MAG(2)+obj.D_MAG(3)];
    
               H = [2*B_mes_BODY'*(eye(3)+D)-2*obj.b_MAG', -S*dEdD+2*J];
    
               R = 300^2;
               A = eye(9);
               Q = 0;
    
               % Prediction
               xk  =  [obj.b_MAG;obj.D_MAG];
               xk1 = A * xk;
    
               obj.P_EKF = A*obj.P_EKF*A' + Q;
    
               % Correction
               hxk         = -1*B_mes_BODY'*(2*D+D*D)*B_mes_BODY + 2*B_mes_BODY'*(eye(3)+D)*obj.b_MAG-norm(obj.b_MAG)^2;
               Observation = norm(B_mes_BODY)^2 - norm(B_true_ECEF)^2;
               K         = obj.P_EKF*H'/(H*obj.P_EKF*H' + R);
               x_est     = xk1 + K*(Observation - H*xk1); % H*xk1
               obj.P_EKF = obj.P_EKF - K*H*obj.P_EKF;
    
               obj.b_MAG = reshape(x_est(1:3),[3,1]);
               obj.D_MAG = reshape(x_est(4:end),[6,1]);

               % Calculate the corrected magnetometer measurement
               D = [obj.D_MAG(1) obj.D_MAG(4) obj.D_MAG(5);...
                    obj.D_MAG(4) obj.D_MAG(2) obj.D_MAG(6);...
                    obj.D_MAG(5) obj.D_MAG(6) obj.D_MAG(3)];

               B_corrected = (eye(3) + D) * B_mes_BODY - obj.b_MAG;
            end

        end

    end
end

