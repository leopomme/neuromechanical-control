% clear all;
clc; set(0,'DefaultFigureWindowStyle','docked'); tic
%clc; close all; set(0,'DefaultFigureWindowStyle','docked'); tic
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modelless learning of feedforward command as a function of time to
% compensate for a VF.
%
% DESIRED TRAJECTORY IN CARTESIAN COORDINATES (column is time)
% NFTraj = [x_position; x_velocity; y_position; y_velocity];
%
% DESIRED TRAJECTORY IN JOINT COORDINATES (column is time)
% qDesired = [angle_shoulder; angle_velocity_shoulder;
%             angle_elbow;    angle_velocity_elbow];
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load reference trajectory for controller and experimental data
load Tutorial4.mat;
% load expt_data.mat;
% Name of experimental conditions in data
FieldName = char('I_NF','II_VF','III_Channel','IV_VF','V_NF');
SizeStruct = zeros(1,size(FieldName,1));
% Calculate block size of each condition
for k=1:size(FieldName,1)
    SizeStruct(1,k) = length(fieldnames(eval(strcat('Data1.(strtrim(FieldName(',num2str(k),',:)))'))));
end
% Parameters for the arm
Mass_S = 3; Mass_E = 3;                   % in kg
Length_S = 0.4; Length_E = 0.3;           % in m
CoM_S = Length_S/2; CoM_E = Length_E/2;   % in m
MoI_S = 0.125; MoI_E = 0.125;             % in kg.m^2
% PD parameters
K = 150; D = 60;
% Channel parameters
KChan = 10000; DChan = 0;                 % in N/m and Ns/m
% Simulation step size
dt = 0.002;                               % in s
% Experiment trial structure
ExptTrialStruct = [0,SizeStruct(1),sum(SizeStruct(1:2)),sum(SizeStruct(1:3)),sum(SizeStruct(1:4)),sum(SizeStruct(1:5))]+1;
% Velocity-dependent force field in CARTESIAN COORDINATES
CurlField = [0 25; -25 0];                % in Ns/m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Simulation structure (NF, VF, channel, VF, NF) for simulation
TrialStructure = ceil(ExptTrialStruct/5); %%%%%%%%%%% CHANGE DIVISION FACTOR FOR SHORTER SIMULATIONS 

Alpha = 0.08; %%%%%%%%%%%% TRY IN THE RANGE [0.08 0.2] 
Gamma = 0.001; %%%%%%%%%%% TRY IN THE RANGE [0.001 0.04 0.1] 

% Euler integration
UpdateSystem = @(x,a)([x(1,1)+dt*x(2,1)+0.5*dt^2*a(1,1); x(2,1)+a(1,1)*dt;
    x(3,1)+dt*x(4,1)+0.5*dt^2*a(2,1); x(4,1)+a(2,1)*dt]);

% Dynamics
JointAccel = @(Torque,JointAngle,JointVel,H)(H\(Torque-[-Mass_E*Length_S*CoM_E*JointVel(2,1)*sin(JointAngle(2,1))*(2*JointVel(1,1)+JointVel(2,1));
    Mass_E*Length_S*CoM_E*JointVel(1,1)^2*sin(JointAngle(2,1))]));

% Allocate memory
uPD = zeros(2,length(NFTraj),TrialStructure(end));
uFF = uPD;
uExt = uPD;
Force = uPD;

% Desired joint trajectories
for i = 1:size(NFTraj,2)
    qDesired(1:2,i) = invKin([Length_S Length_E],[NFTraj(1,i) NFTraj(3,i)]);
end

qDesired = [qDesired(1,:);
    diff(qDesired(1,:))/dt 0;
    qDesired(2,:);
    diff(qDesired(2,:))/dt 0];

q = zeros(4,size(qDesired,2),TrialStructure(end));
q(:,1,:) = repmat(qDesired(:,1),[1,1,TrialStructure(end)]);

% Experiment simulation loop
for Trial=1%%%%%%%%%%%%%% TO BE EXTENDED %%%%%%%%%%%%
    
    % Trial simulation loop
    for i=1:size(NFTraj,2)-1
        % Compute Jacobian
        J = jacobian([Length_S Length_E],[q(1,i,Trial) q(3,i,Trial)]);
        % Compute cartesian velocity
        CartVel = J*[q(2,i,Trial);q(4,i,Trial)];
        
        % External forces
        if Trial>=TrialStructure(2) && Trial<TrialStructure(3) || Trial>=TrialStructure(4) && Trial<TrialStructure(5)
            % Curl velocity dependent field 
            uExt(:,i,Trial) = J'*CurlField*CartVel;
            
        elseif Trial>=TrialStructure(3) && Trial<TrialStructure(4)
            % Forward kinematics
            CartPos = kin([Length_S Length_E], [q(1,i,Trial) q(3,i,Trial)]);
            % Force channel
            uExt(:,i,Trial) = J'*[-KChan*CartPos(1,1)-D*CartVel(1,1);0];
        end
        
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % FEEDFORWARD LEARNING AND FORGETTING
    % TO BE IMPLEMENTED
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        
        % Compute PD control
        uPD(:,i,Trial) = [K*(qDesired(1,i)-q(1,i,Trial))+D*(qDesired(2,i)-q(2,i,Trial));
            K*(qDesired(3,i)-q(3,i,Trial))+D*(qDesired(4,i)-q(4,i,Trial))];
        
        % Calculate mass matrix of arm
        H = mass([Mass_S Mass_E],[Length_S Length_E],[CoM_S CoM_E],[MoI_S MoI_E],[q(1,i,Trial) q(3,i,Trial)]);
        
        % Exit loop if H is singular
        if sum(isnan(H))~=0
            warning('SIMULATION STOPPED AS MASS MATRIX IS SINGULAR');
            break;
        end
        
        % Update system
        q(:,i+1,Trial) = UpdateSystem(q(:,i,Trial),JointAccel(uPD(:,i,Trial)+uFF(:,i,Trial)+uExt(:,i,Trial),[q(1,i,Trial);q(3,i,Trial)],[q(2,i,Trial);q(4,i,Trial)],H));
    end
    
    if sum(isnan(H))~=0
        % Exit loop if H is singular
        break;
    end
end

% Convert to cartesian space
for Trial=1:TrialStructure(end)
    for i=1:size(NFTraj,2)-1
        x(:,i,Trial)= kin([Length_S Length_E],[(q(1,i,Trial)) (q(3,i,Trial))]);
    end
end


% Calculate indices where velocity is maximum during trial
[~, MaxIndex] = max(squeeze(sqrt(q(2,:,:).^2+q(4,:,:).^2)),[],1);

MaxDev = zeros(TrialStructure(end),1);
for Trial=1:TrialStructure(end)
    % Devation at maximum velocity
    %MaxDev(Trial,1) = abs(x(1,MaxIndex(1,Trial),Trial));
    % Maximum deviation of whole movement
    MaxDev(Trial,1) = max(abs(x(1,:,Trial)));
end

toc
%% VECTOR PLOT OF FORCE FIELD

MaxVel = 10;

figure(1); close(1); f1=figure(1); set(f1,'name','1(a)','numbertitle','off'); set(gcf,'color','w');
[xVel,yVel] = meshgrid(-MaxVel:2:MaxVel,-MaxVel:2:MaxVel);

uForceField = CurlField(1,2)*yVel;
vForceField = CurlField(2,1)*xVel;

quiver(xVel,yVel,uForceField,vForceField);

axis square;
xlabel('$\dot{x}$ [m/s]','interpreter','latex','fontsize',20);
ylabel('$\dot{y}$ [m/s]','interpreter','latex','fontsize',20);





%% TRAJECTORY PLOT

% Set color scale for plots
ColorScale = colormap(bone);

figure(3); close(3); f3=figure(3); set(f3,'name','1(b)','numbertitle','off'); set(gcf,'color','w');

% Plot limits in cartesian
PlotLimits = [-0.08 0.08 0.3 0.61];

% For labelling plots
BlockName = char('NF','VF','Channel','VF','NF');

% Allocate memory
MaxDeviationData = zeros(sum(SizeStruct),1);

PickSubject = 1;
% Plot the trajectory of each trial for each Force Field
for Block=1:size(FieldName,1)
    subplot(2,5,Block); hold on;
    for Subject=PickSubject%1:size(Subjects,1)
        for Trial=1:SizeStruct(1,Block)
            TempData = eval(strcat('Data',num2str(Subject),'.(strtrim(FieldName(',num2str(Block),',:))).T',num2str(Trial)));
            plot(TempData(:,1),TempData(:,2),'k');
            MaxDeviationData(Trial+sum(SizeStruct(1,1:Block-1)),Subject) = max(abs(TempData(:,1)));
        end
    end
    
    axis equal; axis(PlotLimits);
    title(strtrim(BlockName(Block,:)),'fontsize',15);
    if Block==1
        ylabel('y (m)','fontsize',12);
    end
    
    xlabel('x (m)','fontsize',12);
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LATERAL FORCE PLOT
subplot(2,1,2); hold on;
% Plot the deviation of each trial
plot(1:size(MaxDeviationData,1),nonzeros(MaxDeviationData(:,Subject)),'.','color',ColorScale(10*(Subject-1)+1,:));

MaxY = 0.09;%1.2*max(max(MaxDeviationData));
% Fill colors to each Force Field: brgrb
hFill(1) = fill([1,1,SizeStruct(1,1)-1,SizeStruct(1,1)-1],[0,MaxY,MaxY,0],'b');
hFill(2) = fill([SizeStruct(1,1)-1,SizeStruct(1,1)-1,sum(SizeStruct(1,1:2))-1,sum(SizeStruct(1,1:2))-1],[0,MaxY,MaxY,0],'r');
hFill(3) = fill([sum(SizeStruct(1,1:2))-1,sum(SizeStruct(1,1:2))-1,sum(SizeStruct(1,1:3))-1,sum(SizeStruct(1,1:3))-1],[0,MaxY,MaxY,0],'g');
hFill(4) = fill([sum(SizeStruct(1,1:3))-1,sum(SizeStruct(1,1:3))-1,sum(SizeStruct(1,1:4))-1,sum(SizeStruct(1,1:4))-1],[0,MaxY,MaxY,0],'r');
hFill(5) = fill([sum(SizeStruct(1,1:4))-1,sum(SizeStruct(1,1:4))-1,sum(SizeStruct(1,1:5)),sum(SizeStruct(1,1:5))],[0,MaxY,MaxY,0],'b');

xlim([1,size(MaxDeviationData,1)]);
ylim([0,1]*MaxY);
set(hFill,'facealpha',0.2,'edgealpha',0)

xlabel('Trials','fontsize',15);
ylabel('Max absolute lateral deviation (m)','fontsize',15);

legend(hFill(1:3),'NF','VF','Channel','location','northeast'); legend boxoff;


%% SIMULATION PLOTS

% TRAJECTORIES

figure(4); close(4); f4=figure(4); set(f4,'name','2(a)','numbertitle','off'); set(gcf,'color','w');

subplot(2,5,1);
plot(squeeze(x(1,:,1:TrialStructure(2)-1)),squeeze(x(2,:,1:TrialStructure(2)-1)));
xlabel('x (m)','fontsize',12);
ylabel('y (m)','fontsize',12);
axis equal; axis(PlotLimits);
subplot(2,5,2);
plot(squeeze(x(1,:,TrialStructure(2):TrialStructure(3)-1)),squeeze(x(2,:,TrialStructure(2):TrialStructure(3)-1)));
xlabel('x (m)','fontsize',12);
axis equal; axis(PlotLimits);
subplot(2,5,3);
plot(squeeze(x(1,:,TrialStructure(3):TrialStructure(4)-1)),squeeze(x(2,:,TrialStructure(3):TrialStructure(4)-1)));
xlabel('x (m)','fontsize',12);
axis equal; axis(PlotLimits);
subplot(2,5,4);
plot(squeeze(x(1,:,TrialStructure(4):TrialStructure(5)-1)),squeeze(x(2,:,TrialStructure(4):TrialStructure(5)-1)));
xlabel('x (m)','fontsize',12);
axis equal; axis(PlotLimits);
subplot(2,5,5);
plot(squeeze(x(1,:,TrialStructure(5):end)),squeeze(x(2,:,TrialStructure(5):end)));
xlabel('x (m)','fontsize',12);
axis equal; axis(PlotLimits);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAXIMUM ABSOLUTE LATERAL DEVIATION
subplot(2,1,2); hold on;

plot(1:TrialStructure(end),MaxDev,'--ko','markersize',8,'linewidth',0.1);

hFill(1) = fill([1,1,TrialStructure(2)-1,TrialStructure(2)-1],[0,MaxY,MaxY,0],'b');
hFill(2) = fill([TrialStructure(2)-1,TrialStructure(2)-1,TrialStructure(3)-1,TrialStructure(3)-1],[0,MaxY,MaxY,0],'r');
hFill(3) = fill([TrialStructure(3)-1,TrialStructure(3)-1,TrialStructure(4)-1,TrialStructure(4)-1],[0,MaxY,MaxY,0],'g');
hFill(4) = fill([TrialStructure(4)-1,TrialStructure(4)-1,TrialStructure(5)-1,TrialStructure(5)-1],[0,MaxY,MaxY,0],'r');
hFill(5) = fill([TrialStructure(5)-1,TrialStructure(5)-1,TrialStructure(end),TrialStructure(end)],[0,MaxY,MaxY,0],'b');

xlim([1,TrialStructure(end)]);
ylim([0,1]*MaxY);
set(hFill,'facealpha',0.2,'edgealpha',0)

xlabel('Trials','fontsize',15);
ylabel('Max absolute lateral deviation (m)','fontsize',15);

legend(hFill(1:3),'NF','VF','Channel','location','northeast'); legend boxoff;



