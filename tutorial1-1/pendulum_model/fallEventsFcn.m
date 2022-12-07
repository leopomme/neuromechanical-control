function [fall_event,isterminal,direction] = fallEventsFcn(t,theta)
    % fallEventsFcn checks if the pendulum has fallen and stops the
    % integration when it does. 
    fall_event = theta(1) > pi/2 || theta(1)< -pi/2 ; % condition to stop the simulation
    isterminal = 1;  % Halt integration 
    direction = 0;   % The zero can be approached from either direction
end