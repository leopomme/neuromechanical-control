function ldot = muscle_model_ode(t,l,u)
% Muscle parameters
    alpha = -0.0218;
    k_0 = 810.8;  
    k = 1621.6;
    damping_ratio = 0.26;
    m = 0.3; % we consider a mass attached to the muscle
    delta_0 = 2*damping_ratio*sqrt(m*k_0);
    delta = 2*damping_ratio*sqrt(m*k);
    l_0 = 0.10;
    g = 9.81;
    
    k_total = (k_0 + k*u);
    l_update = (l_0 + alpha*u -l(1));
    d_total = (delta_0 + delta * sqrt(u));
    ldot = [ l(2);
             (1/m)*(k_total*l_update-d_total*l(2)) + g; 
            ];
  
end

