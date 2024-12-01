function [J] = cost_interplanetary(args)
    e = args(1);
    h = args(2);
    r_final = evalin('base','mars_prm_st.a_km');
    r_init  = evalin('base','earth_prm_st.a_km');
    mu_sun  = evalin('base','sun_prm_st.mu_km3_s2');
    w_mars  = evalin('base','w_mars_deg_s') * pi / 180;
    mars_i  = evalin('base','mars_prm_st.inclination_rad');
    phi     = evalin('base','phase_angle_deg') * pi / 180;
    plot_data = evalin('base','plot_trans_orbit');

    T = 0;

    theta_0 = 0;
    time_increment_s = 10;

    r     = r_init;
    theta = theta_0; 

    theta_mars = phi;
    w_mars_helio = w_mars * cos(mars_i);

    distance_init = sqrt((r_final * cos(phi) - r_init)^2 + (r_final * sin(phi) - r_init));

    distance = distance_init;
    if (plot_data)
        x_celes_arr = [];
        y_celes_arr = [];
    
        x_mars_arr = [];
        y_mars_arr = [];
        dist_arr   = [];
        T_arr      = [];
    
    end



    while (theta<=pi)
        theta_dot = h / r^2;
        theta     = theta + theta_dot * time_increment_s;

        r = (h^2/ mu_sun) * (1 / (1 + e * cos(theta)));

        x_celestial = r * cos(theta);
        y_celestial = r * sin(theta);

        % Mars 

        theta_mars = theta_mars + w_mars_helio * time_increment_s;
        x_mars     = r_final * cos(theta_mars);
        y_mars     = r_final * sin(theta_mars);

        d_ = sqrt((x_celestial - x_mars)^2 + (y_celestial - y_mars)^2);

        if (d_<distance)
            distance = d_;
        end

        if (plot_data)
            x_celes_arr = [x_celes_arr,x_celestial];
            y_celes_arr = [y_celes_arr,y_celestial];
            x_mars_arr = [x_mars_arr,x_mars];
            y_mars_arr = [y_mars_arr,y_mars];
            dist_arr   = [dist_arr,d_];
            T_arr      = [T_arr,T];
        end

        T = T + time_increment_s;


    end
    J       = distance;
    delta_J = J / distance_init * 100;
    assignin('base','time_to_mars',T);

    if (plot_data)
        figure;hold on;
        plot3(x_celes_arr,y_celes_arr,T_arr / (3600 * 24));
        plot3(x_mars_arr,y_mars_arr,T_arr   / (3600 * 24));
        xlim([-r_final*1.1, r_final *1.1])
        ylim([-r_final*1.1, r_final *1.1])
        zlim([0, 300])
        legend("Celes","Mars")
        grid minor
    
        figure;
        plot(T_arr / (3600 * 24),dist_arr)
    end
end