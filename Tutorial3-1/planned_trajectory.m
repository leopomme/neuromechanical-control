% Compute the planned trajectory in the joint space

function [qr,qr_dot,x] = planned_trajectory(l,dt,T,q_start)

%% Desired Trajectory 
T_samples  = fix(T/dt);
qr = zeros(T_samples,2);
qr_dot = zeros(T_samples,2);

% Initial posture
qr(1,:) = q_start; % in radian
qr_dot(1,:) = [0,0];

% Forward kinematics to calculate initial conditions
x = zeros(size(qr));
x(1,:) = kin(l,qr(1,:));

% Kinematic transformation
k = 1;
for t = dt : dt : T

    tT = t/T;
    g = 6*(tT)^5 - 15*(tT)^4 + 10*(tT)^3;
    x(k+1,1) = -0.2605 + 0.11*g;
    x(k+1,2) = 0.0915 + 0.5*g;
    
    % Calculating joint angular velocities
    v = ( x(k+1,:) - x(k,:) ) / dt;
    q = qr(k,:);
    J= Jacobian(l,q);
    q_dot = J \ v';
    
    % Numerical integration 
    qr(k+1,1) = q(1) + dt * q_dot(1);
    qr(k+1,2) = q(2) + dt * q_dot(2);
    qr_dot(k+1,:) = q_dot;
           
    k = k + 1;   
    
end
