% Tutorial Muscle Behavior

% Muscle parameters
alpha = -0.0218;
k_0 = 810.8;  
k = 1621.6;
m = 0.2;
damping_ratio = 0.26;
delta_0 = 2*damping_ratio*sqrt(m*k_0);
delta = 2*damping_ratio*sqrt(m*k);
l_0 = 0.10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PART 1: Muscle Length for different muscle activation, 
%%% T = 0 (what happens if you apply a constant non null tension?)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms l(t);
T = 0;
final_length= [];
tspan = [0 0.1];
% Plot the muscle length vs activation
figure()
u_range = [0:0.2:1];
for u = u_range
    K_tot = k_0 + k*u;
    D_tot = delta_0 + delta * sqrt(u);
    
    ode = diff(l,t) == (K_tot*(l_0+alpha*u-l)-T)/D_tot;
    cond = l(0) == l_0;    
    ySol(t) = dsolve(ode,cond);
    fplot(ySol,tspan,'DisplayName',['Activation = ' num2str(u)]);
    hold on;
    final_length_u = ySol(3);
    final_length = [final_length,final_length_u];
end
xlabel('time s');
ylabel('muscle Length [m]');
title('muscle length vs time');
legend;

% Plot the muscle length vs time

figure()
plot(u_range,final_length,'*--');
xlabel('muscle activation');
ylabel('muscle Length [m]');
title('muscle length vs activation');

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PART 2: Muscle Tension for different muscle activation at constant
%%% length (isometric condition). 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
l_t0_val = [l_0*0.8 ,l_0*0.9, l_0 ,l_0*1.1 ,l_0*1.2];
u_range = [0 1];
figure()
for l_t0 = l_t0_val
    %l_t0 = l_t0_val(i);
    fplot(@(u) (k_0 + k*u)*(l_0 + alpha*u - l_t0),u_range,'DisplayName',['muscle length = ' num2str(l_t0)]);
    hold on;
end
legend
xlabel('muscle activation');
ylabel('Tension [N]');
title('Muscle tension vs activation');
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PART 3: Muscle Behavior after a stretch for different level of
%%% activation.  Descibe this plot in Question 2! 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms l(t) ;
activation_level = [0 0.5 1];
l_init = 1.3*l_0;
tspan = [0 0.5];
figure()
ax1 = subplot(2,1,1);
ax2 = subplot(2,1,2);

for i = 1:length(activation_level)
    u_i = activation_level(i);
    [t f]= ode45(@(t,l) muscle_model_ode(t,l,u_i),tspan,[l_init;0]);
    
    subplot(ax1)
    plot(t,f(:,1),'DisplayName',['u = ' num2str(u_i)]);
    hold on;
    
    subplot(ax2)
    plot(t,f(:,2),'DisplayName',['u = ' num2str(u_i)]);
    hold on;
end

xlabel('time [s]');
ylabel(ax1,'Muscle Length [m]');
ylabel(ax2,'Velocity [m/s]');
title(ax1,'Muscle length vs time');
title(ax2,'Velocity vs time');
legend(ax1);
legend(ax2);
