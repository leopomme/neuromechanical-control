
%% Pendulum - Muscle  Model
% Create your own pendulum_muscle_equation function. It should be solvable
% with ode45 and the solution you return both theta and theta dot. 

% PART 1:  Study how co-contraction influences the stability and position/orientation of the forearm. To do this use three values of co-contraction u=0, u=0.5 and u=1. 
% Plot the evolution of theta and thetadot for two different set of initial conditions, for which the system:
%       1) Does not fall to the ground for both small (u = 0.5) and large (u = 1) activation level; 
%       2) Does not fall to the ground for large (u = 1) activation level and falls for smaller ones (u = 0.5);

figure()
ax1 = subplot(2,1,1);
ax2 = subplot(2,1,2);

%activation
u_set =  [0,0;0.5,0.5;1,1];
tspan = [0 5];
n = size(u_set);

% Set the initial conditions
initial_cond = %% INSERT YOUR CODE %%
options = odeset('Events',@ fallEventsFcn);

for i = 1:n(1)
    u_set_i = u_set(i,:); 
    
    % Compute theta and thetadot using your own pendulum_muscle_equation
    [t,f] = ode45(@(t,theta) pendulum_muscle_equation(t,theta,u_set_i(1),u_set_i(2)),tspan,initial_cond,options); 
    
    subplot(ax1)
    plot(t(:),f(:,1)*180/pi,'DisplayName',['u1 =' num2str(u_set_i(1)) '; u2 = ' num2str(u_set_i(2))]);
    hold on;
    subplot(ax2)
    plot(t(:),f(:,2),'DisplayName',['u1 =' num2str(u_set_i(1)) '; u2 = ' num2str(u_set_i(2))]);
    hold on;
end
ylabel(ax1,'Theta [°]');
ylabel(ax2,'Thetadot [rad/s]');
xlabel(ax2,'Time [s]');
title(ax1,'Angle');
title(ax2,'Velocity');
legend(ax1);
legend(ax2);
%% Pendulum - Muscle  Model PART 2
% PART 2: Keeping the flexor muscle behaviour relaxed, and using constant activation of the extensor, 
% simulate the behaviour of hitting a nail with a hammer. 
% Provide a plot of the pendulum momentum, the velocity and the position. 
% At what level of activation do you need to stimulate your muscle to hit the nail at 90° from the resting position in less than 0.5 s ?

figure()
ax1 = subplot(3,1,1);
ax2 = subplot(3,1,2);
ax3 = subplot(3,1,3);
u_set =  [0.,0.2;0.,0.5;0,1];
tspan = [0 2];
n = size(u_set);
initial_cond = [0;0.1];
m = 1;
r = 0.3 ;
g = 9.81;

for i = 1:n(1)
    u_set_i = u_set(i,:); 
    [t,f,te,ye,ie] = ode45(@(t,theta) pendulum_muscle_equation(t,theta,u_set_i(1),u_set_i(2)),tspan,initial_cond,options); 
    pendulum_momentum = %% INSERT YOUR CODE %%%
  
    subplot(ax1)
    plot(t(:),pendulum_momentum,'DisplayName',['u1 =' num2str(u_set_i(1)) '; u2 = ' num2str(u_set_i(2))]);
    hold on;
    subplot(ax2)
    plot(t(:),f(:,2),'DisplayName',['u1 =' num2str(u_set_i(1)) '; u2 = ' num2str(u_set_i(2))]);
    hold on;
    subplot(ax3)
    plot(t(:),f(:,1)*180/pi,'DisplayName',['u1 =' num2str(u_set_i(1)) '; u2 = ' num2str(u_set_i(2))]);
    xline(te,'HandleVisibility','off')
    yline(90,'HandleVisibility','off');
    text(te,80,num2str(te))  
    hold on;
end

ylabel(ax1,'Pendulum Momentum ');
ylabel(ax3,'Theta [°]');
ylabel(ax2,'Thetadot [rad/s]');
xlabel(ax2,'Time [s]');
title(ax1,'Pendulum Momentum');
title(ax2,'Velocity');
title(ax3,'Theta');
legend ;

% 
%% Pendulum animation - Verify your mdoel behaviour
% Simulate the muscle using your pendulum_muscle_equation function, you can
% verify the behaviour of your model ! You can also have fun and improve the animation (OPTIONAL) ! 
stim_time = 10;
initial_cond = [0.0;0.0];
activation = [0,0];
f = ode45(@(t,theta) pendulum_muscle_equation(t,theta,activation(1),activation(2)),[0 stim_time],initial_cond,options);
animate_pendulum(f,stim_time)
