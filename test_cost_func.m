% phase_angle_deg = 44.3;
rp = earth_prm_st.a_km;
ra = mars_prm_st.a_km;

e = (ra - rp) / (ra + rp);
h = sqrt(ra * sun_prm_st.mu_km3_s2*(1+e*cosd(180)));

% [J,delta_J] = cost_interplanetary([e,h])
plot_trans_orbit = 0;
x0 = [e,h];
options = optimset('MaxFunEvals',10000,'MaxIter',10000,'TolX',0.1e6);
[x,fval,exitflag,output] = fminsearch('cost_interplanetary',x0,options)


%% 
plot_trans_orbit = 1;
[J] = cost_interplanetary(x)
