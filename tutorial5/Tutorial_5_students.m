
clear all; clc; close all;

% Discretization of continuous system
dt = 0.01;

% Generate time array
t = 0:dt:4*pi;

%%%%%%% Kalman filter matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% State transition matrix
A = [1 dt dt^2/2;
     0  1     dt;
     0  0     1];

% System noise and covariance matrix
SigmaQ = 0.3;
Q=[ SigmaQ^6/36    SigmaQ^5/12   SigmaQ^4/6
    SigmaQ^5/12    SigmaQ^4/4    SigmaQ^3/2
    SigmaQ^4/6     SigmaQ^3/2    SigmaQ^2];

% Observation matrix
C = [1 0 0];

% Observation noise and covariance matrix
Sigma = 1.5;
R=(Sigma^2);

% Initialize state and error covariance matrices
xInit = zeros(3,1);
PInit = diag([1 1 1]).*10^5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Generate signal and noise corrupted signal
Signal = 40*sin(t);
SignalNoisy = Signal + sqrt(R)*randn(size(R,1),length(Signal));

% LQE function
%  x = KalmanFilter(A,C,Q,R,xInit,PInit,%insert noisy signal here);

%% Plots
set(0,'DefaultFigureWindowStyle','docked')

figure(1);set(gcf,'color','white');
hold on;
plot(t,Signal,'--k','linewidth',2.5);
plot(t,SignalNoisy(1,:),'r');
% plot(t,%insert estimate signal here,'b','linewidth',1.5);

legend('True position','Noisy position','Filtered Signal');

xlabel('Time (s)','fontsize',15);
ylabel('Position (cm)','fontsize',15);



