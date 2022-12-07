
clear all; clc; 
close all;



%% Initialisation

% Define sampling rate
dt = 0; % DEFINE HERE THE SAMPLING RATE

% Define movement duration
T = 0; % DEFINE HERE THE MOVEMENT DURATION

T_samples = fix(T/dt);

% Kinematic parameters
l = [0.31,0.34]; % m
% Dynamic parameters
m = [1.93,2.04]; % kg
I = [0.0141,0.0188]; %Inertia moments kg*m^2
cL = [0.165,0.2]; % m

% Initial joint angles
q_start = [0, 0]; % WRITE HERE THE INITIAL JOINT ANGLES

%% Compute reference (planned) trajectory
[qr,qr_dot,xp] = planned_trajectory(l,dt,T,q_start);

%% Initialisation

q = zeros(T_samples,2);
q(1,:) = q_start;
qdot = zeros(T_samples,2);
qdot(1,:) = [0,0];

% Integration functions

UpdateAngle = @(q,qdot)([q(1)+dt*qdot(1);
                       q(2)+dt*qdot(2)]); 

UpdateVel = @(qdot,qddot)([qdot(1)+dt*qddot(1);
                       qdot(2)+dt*qddot(2)]); 
                   
JointAccel = @(Torque,H,Cqdot)(H\(Torque-Cqdot)); 

% PD constants respectively proportional and derivative
% DEFINE BELOW THE PD CONSTANTS
  Kp=0;
  Kd=0;
  
for i=1:T_samples
    
   % WRITE HERE THE CONTROLLER EQUATION/TORQUE TERMS
   % ....
   % .... 
   Torque = [0 0]';
   % ....
   % .... 
    
    % Compute dynamics
    H= mass(m,l,cL,I,q(i,:));
    Cqdot = coriolis(I,m,l,cL,q(i,:),qdot(i,:));
    qddot = JointAccel(Torque,H,Cqdot); 

    % Movement integration
    qdot(i+1,:) = UpdateVel(qdot(i,:),qddot);
    q(i+1,:) = UpdateAngle(q(i,:),qdot(i+1,:));
    
end

for i = 1:size(q,1)

    X= kin(l,q(i,:));
    x(i) = X(1);
    y(i) = X(2);

    X= kin(l,qr(i,:));
    xr(i) = X(1);
    yr(i) = X(2);

end

%% Plots

t = 0:dt:T;

% Angle evolution
figure,
plot(t,q(:,1)*180/pi,'r','linewidth',2)
hold on
plot(t,qr(:,1)*180/pi,'m--','linewidth',2)
hold on
plot(t,q(:,2)*180/pi,'b','linewidth',2)
hold on
plot(t,qr(:,2)*180/pi,'c--','linewidth',2)
xlabel('time [s]')
ylabel('angle [degree]')
legend('shoulder act','shoulder ref','elbow act','elbow ref')
set(gca,'fontsize',24)

% Position evolution
figure,
plot(t,x,'r','linewidth',2)
hold on
plot(t,xr,'m--','linewidth',2)
hold on
plot(t,y,'b','linewidth',2)
hold on
plot(t,yr,'c--','linewidth',2)
xlabel('time [s]')
ylabel('position [m]')
legend('x','x ref','y','y ref','Location','Best')
% legend('elbow')
set(gca,'fontsize',24)

% Trajectory
figure, 
plot(x,y,'k','linewidth',2)
hold on
plot(xr,yr,'k--','linewidth',2) 
axis equal
legend('Actual trajectory', 'Reference trajectory')

