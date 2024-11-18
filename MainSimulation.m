classdef MainSimulation
    % Spacecraft parameters comes from ChatGPT :)
    % It is not reliable.
   properties
      spacecraft_pose_ecef_a_km 
      spacecraft_vel_ecef_a_km_s
      sample_time_s                   
      earth_prm_st                
      q_ecef2b                   % Body frame orientation relative to the ecef
      q_b2ecef
      spacecraft_mass_kg  
      altitude_km
      angle_of_attack_rad
      flight_path_angle_rad
      rho_kg_m3

      % The data below is just for simulation
      area_m2
      Cd_coef
      
   end
   methods
       function obj = MainSimulation(pose_ecef_km,vel_ecef_km_s,sample_time_s,earth_prm_st,q_ecef2b,mass_kg)
        obj.spacecraft_pose_ecef_a_km  = pose_ecef_km;
        obj.spacecraft_vel_ecef_a_km_s = vel_ecef_km_s;
        obj.sample_time_s              = sample_time_s;
        obj.earth_prm_st               = earth_prm_st;
        obj.q_ecef2b                   = q_ecef2b;
        obj.q_b2ecef                   = quatinv(q_ecef2b);
        obj.spacecraft_mass_kg         = mass_kg;

        obj.area_m2                    = pi * 2.5^2;
        obj.Cd_coef                    = 0.2;
       end

       function obj = simulate(obj, thrust_N,w_b_a_rad_s)

         position_magnitude_ecef_km = sqrt(obj.spacecraft_pose_ecef_a_km(1)^2 + ...
                                           obj.spacecraft_pose_ecef_a_km(2)^2 + ...
                                           obj.spacecraft_pose_ecef_a_km(3) );


         % Drag coefficient will be calculated
         CD                          = obj.calculate_drag_coefficient(obj.angle_of_attack_rad);
         vel_m_s                     = obj.calculate_vel() * 1000;
         drag_force_mag              = 0.5 * obj.rho_kg_m3 * vel_m_s^2 * obj.area_m2 * CD;
         drag_vector_a_ecef_N        = [0.0,0.0,0.0]; % Equals to zero for now  


         gravity_vector_a            = -1 * obj.spacecraft_pose_ecef_a_km / position_magnitude_ecef_km;
         thrust_vector_a_ecef_N      =  quatrotate(obj.q_b2ecef,[thrust_N,0.0, 0.0]);
         acceleration_vector_a_km_s2 = ((obj.earth_prm_st.mu_km3_s2 / (position_magnitude_ecef_km^2) * gravity_vector_a)  ) + ... % Gravitational component
                                       ((thrust_vector_a_ecef_N / obj.spacecraft_mass_kg) / 1000.0)                         + ... % Thrust component
                                       ((drag_vector_a_ecef_N   / obj.spacecraft_mass_kg) / 1000.0);                              % Drag component (equals to zero for now)



         % Propogation part
         % Orientation update 
         q_w_b = [0,w_b_a_rad_s(1),w_b_a_rad_s(2),w_b_a_rad_s(3)];
         q_ecef2b_dot = 0.5 * quatmultiply(obj.q_ecef2b,q_w_b);
         obj.q_ecef2b = obj.q_ecef2b + q_ecef2b_dot * obj.sample_time_s;
         obj.q_ecef2b = quatnormalize(obj.q_ecef2b);
         obj.q_b2ecef = quatinv(obj.q_ecef2b); 

         % Altitude
         obj.altitude_km = position_magnitude_ecef_km - obj.earth_prm_st.radius_km;

         % Position update
         obj.spacecraft_pose_ecef_a_km = obj.spacecraft_pose_ecef_a_km + obj.spacecraft_vel_ecef_a_km_s * obj.sample_time_s;

         % Velocity update
         obj.spacecraft_vel_ecef_a_km_s = obj.spacecraft_vel_ecef_a_km_s + acceleration_vector_a_km_s2 * obj.sample_time_s;

         obj.calculate_estimation_data(gravity_vector_a);

       end

       function vel_km_s = calculate_vel(obj)
         vel_km_s = sqrt(obj.spacecraft_vel_ecef_a_km_s(1)^2 + ...
                         obj.spacecraft_vel_ecef_a_km_s(2)^2 + ...
                         obj.spacecraft_vel_ecef_a_km_s(3) );
       end

       function rho_kg_m3 = calculate_air_density(obj)
           % Fit data from ISA model. Delta ISA : 0 deg
         p1 =  -2.943e-14 ;
         p2 =   3.359e-09 ;
         p3 =  -0.0001119 ;
         p4 =       1.218 ;
         x  = obj.altitude_km * 1000;
         rho_kg_m3 = p1*x^3 + p2*x^2 + p3*x + p4;
         rho_kg_m3 = max(0.0,rho_kg_m3);

       end

       function CD = calculate_drag_coefficient(alpha_rad)
           CD = 0.2 + 0.1146 * alpha_rad;
       end

       function obj = calculate_estimation_data(obj,gravity_vector_a)
         % Angle of attack calculation
         body_x_vector_ecef = quatrotate(obj.q_b2ecef,[1.0,0.0, 0.0]);

         velocity_magnitude = sqrt(obj.spacecraft_vel_ecef_a_km_s(1)^2 + ...
                                   obj.spacecraft_vel_ecef_a_km_s(2)^2 + ...
                                   obj.spacecraft_vel_ecef_a_km_s(3) );

         velocity_direction_vec_ecef = obj.spacecraft_vel_ecef_a_km_s / velocity_magnitude;

         cp = cross(body_x_vector_ecef,velocity_direction_vec_ecef);
         obj.angle_of_attack_rad = asin(sqrt(cp(1)^2 + cp(2)^2 + cp(3)^2));

         % Flight path angle
         cp = cross(gravity_vector_a, velocity_direction_vec_ecef);
         obj.flight_path_angle_rad = pi/2 - asin(sqrt(cp(1)^2 + cp(2)^2 + cp(3)^2));

         % Air density
         obj.rho_kg_m3 = obj.calculate_air_density();
    
       end
   end
end