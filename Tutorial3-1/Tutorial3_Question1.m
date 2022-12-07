
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


for i=1:T_samples
    
   % WRITE HERE THE TORQUE TERMS
   % ....
   % .... 
   Torque = [0 0]';
   % ....
   % .... 
    
    % Compute Dynamics
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
end

%% Plots

t = 0:dt:T;

% Question 1.b/1.c
figure 
plot(t,q(:,1)*180/pi,'linewidth',2);
hold on
plot(t,q(:,2)*180/pi,'r','linewidth',2);
legend('shoulder','elbow','Location','SouthEast')
xlabel('time [s]')
ylabel('angle [degree]')
set(gca,'fontsize',24)
