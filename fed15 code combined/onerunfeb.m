
clc
clear all
format long 
VxImpact =0;
inclinationImpact = 10; %degrees
yawImpact = 45; %degrees

% angle = (inclinationImpact - 0.0042477)/1.3836686;
% rollImpact = -angle; %degrees
% pitchImpact = -angle; %degrees

%% Declare Globals
global g
global timeImpact
global globalFlag

%% Initialize Simulation Parameters
ImpactParams = initparams_navi;

SimParams.recordContTime = 0;
% SimParams.useFaesslerRecovery = 1;%Use Faessler recovery
SimParams.useRecovery = 1;
% SimParams.useRecovery2 = 1;
SimParams.timeFinal =0.1;
tStep = 1/200;%1/200;

ImpactParams.wallLoc = 30;%1.5;
ImpactParams.wallPlane = 'YZ';
ImpactParams.timeDes = 1; %Desired time of impact. Does nothing
ImpactParams.frictionModel.muSliding = 0.3;%0.3;
ImpactParams.frictionModel.velocitySliding = 1e-4; %m/s
timeImpact = 10000;
timeStabilized = 10000;

%% Initialize Structures
IC = initIC;
Control = initcontrol;
PropState = initpropstate;
Setpoint = initsetpoint;

[Contact, ImpactInfo] = initcontactstructs;
localFlag = initflags;

ImpactIdentification = initimpactidentification;

%% Set initial Conditions

%%%%%%%%%%%% ***** SET INITIAL CONDITIONS HERE ***** %%%%%%%%%%%%%%%%%%%%%%
twist.posnDeriv(1) = VxImpact; %World X Velocity at impact      %%%
% IC.attEuler = [deg2rad(rollImpact);...                                  %%%
%                deg2rad(pitchImpact);...                                 %%%
%                deg2rad(yawImpact)];                                     %%%
% IC.posn = [ImpactParams.wallLoc-0.32;0;10]; 
% IC.attEuler=[0.5;0.5;0];
IC.attEuler=[0.5;0.5;0];
IC.posn=[0;0;10];
Setpoint.posn(3) = IC.posn(3);                                          %%%
xAcc = 0;                                                               %%%
%%%%%%%%%%% ***** END SET INITIAL CONDITIONS HERE ***** %%%%%%%%%%%%%%%%%%%

rotMat = quat2rotmat(angle2quat(-(IC.attEuler(1)+pi),IC.attEuler(2),IC.attEuler(3),'xyz')') % Rrotmat I to body

% [IC.posn(1), VxImpact, SimParams.timeInit, xAcc ] = getinitworldx( ImpactParams, Control.twist.posnDeriv(1),IC, xAcc);
SimParams.timeInit = 0; %% comment out if using getinitworldx()

Setpoint.head = IC.attEuler(3);
Setpoint.time = SimParams.timeInit;
Setpoint.posn(1) = IC.posn(1);
Trajectory = Setpoint;

IC.linVel =  rotMat*[VxImpact;0;0]; %rotmat takes from intertial to body

Experiment.propCmds = [];
Experiment.manualCmds = [];

globalFlag.experiment.rpmChkpt = zeros(4,1);
globalFlag.experiment.rpmChkptIsPassed = zeros(1,4);

[IC.rpm, Control.rpm] = initrpm(rotMat, [xAcc;0;0]); %Start with hovering RPM
% Control.rpm=[5555;5555;5555;5555];
Control.rpm=[0;0;0;0];
PropState.rpm = IC.rpm;

%% Initialize state and kinematics structs from ICs
[state, stateDeriv] = initstate(IC, xAcc);
state
[Pose, Twist] = updatekinematics(state, stateDeriv);
 state
%% Initialize sensors
Sensor = initsensor(state, stateDeriv);

Hist = inithist(SimParams.timeInit, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor);


                                           %% %% dandrea %% %%
% for iSim = SimParams.timeInit:tStep:SimParams.timeFinal-tStep                                           
                                           
 [Control] = calculatedesacceleration(Pose, Twist); 
 [Control] = controllerfailrecover(tStep, Pose, Twist, Control);

 %% Propagate Dynamics
 iSim =0;
%  options = getOdeOptions();
  options = odeset('RelTol',1e-15);
  state
 [tODE,stateODE] = ode45(@(tODE, stateODE) dynamicsystem(tODE,stateODE, ...
                                                            tStep,Control.rpm,ImpactParams,PropState.rpm, ...
                                                            Experiment.propCmds),[iSim iSim+tStep],state,options);
% tODE=0;
% [tODE,statederiv]=dynamicsystem(Control.rpm,ImpactParams,PropState.rpm,Experiment.propCmds,[iSim iSim+tStep],state,options);
%     statederiv
% q = [state(10);state(11);state(12);state(13)]/norm(state(10:13));
% rotMat = quat2rotmat(q);
% 
% state = reshape(state,[max(size(state)),1]); %make sure state is column vector
% stateDeriv = zeros(13,1); %initialize stateDeriv
% rpmControl = Control.rpm;
% % rpmDeriv=0;
% % PropState.rpmDeriv = rpmDeriv;
% %% Calculate External Forces and Moments
% g = 9.81;   % gravity
% m =  1.095;
% Kr= 5*10^-3;
% Ixx = 0.01121976;
% Iyy = 0.01122668;
% Izz = 0.021082335;
% Ixy = -5.62297e-05;
% Iyz = -4.4954e-06;
% Izx = -1.418e-08;  
% Izzp=2.20751e-5;
%     IzzT=Izz +Izzp;
% IB = [Ixx Ixy Izx;Ixy Iyy Iyz;Izx Iyz Izz]; 
%% Save Global Variables locally
% global g m I  Jr PROP_POSNS %Inertial and Geometric Parameters
% Kt_fiona = 8.7e-8; 
% Dt = 9.61e-10;
% Fg = rotMat*[0;0;-m*g]; %Gravity 
% V = 0;
% % Fa = Tv*[-0.5*AERO_DENS*V^2*AERO_AREA*Cd;0;0]; %Aerodynamic
% Fa=0;
% Ft = [0;0;-Kt_fiona*sum(rpmControl.^2)]; %Thruster

% totalContactForce = sum(normalForceBody,2) + sum(tangentialForceBody,2);
% totalContactMoment = sum(contactMomentBody,2);
% totalContactMoment=[0;0;0];
% Mx = -Kt*PROP_POSNS(2,:)*(rpmControl.^2)-state(5)*Jr*sum(rpm2rad(rpmControl)) + totalContactMoment(1) ;
% My = Kt*PROP_POSNS(1,:)*(rpmControl.^2)+state(4)*Jr*sum(rpm2rad(rpmControl)) + totalContactMoment(2) ;
% Mz =  [-Dt Dt -Dt Dt]*(rpmControl.^2)-Kr*state(6)-Jr*sum(rpmDeriv) + totalContactMoment(3);
% Mx = -Kt_fiona*PROP_POSNS(2,:)*(rpmControl.^2)-state(5)*Jr*sum(rpm2rad(rpmControl));
% My = Kt_fiona*PROP_POSNS(1,:)*(rpmControl.^2)+state(4)*Jr*sum(rpm2rad(rpmControl));
% Mz =  [-Dt Dt -Dt Dt]*(rpmControl.^2)-Kr*state(6);

%% Update State Derivative
% stateDeriv(1:3) = (Fg + Fa + Ft + totalContactForce - m*cross(state(4:6),state(1:3)))/m;
% stateDeriv(1:3) = (Fg + Fa + Ft  - m*cross(state(4:6),state(1:3)))/m
% stateDeriv(4:6) = inv(IB)*([Mx;My;Mz]-cross(state(4:6),IB*state(4:6)))
% stateDeriv(7:9) = rotMat'*state(1:3)
% stateDeriv(10:13) = -0.5*quatmultiply([0;state(4:6)],q)

    % Reset contact flags for continuous time recording        
%     globalFlag.contact = localFlag.contact;
%    x=R1
       
%% Record History
%     if SimParams.recordContTime == 0
        
%  [stateDeriv, Contact, PropState] = dynamicsystem(tODE(end),stateODE(end,:), ...
%                                                          tStep,Control.rpm,ImpactParams, PropState.rpm, ...
%                                                          Experiment.propCmds)      ; 
    state = stateODE(end,:)'
    t = tODE(end);
    [Pose, Twist] = updatekinematics(state, stateDeriv);
    
    %% Comparing to dandrea
    R_F2D=[ sqrt(2)/2    sqrt(2)/2            0;
        sqrt(2)/2     -sqrt(2)/2          0;
        0               0                -1];
    pqrf=state(4:6)
    pqrd=R_F2D*pqrf
    quatf=state(10:13)
    R_I2Ff = quat2rotmat(quatf); %from inertial to fiona
    R_I2Df=R_F2D*R_I2Ff;
    R_D2If=R_I2Df';
    q_Dandreafinal=rotmat2quat(R_D2If)
    uvwf=state(1:3)
    uvwd=R_F2D*uvwf
    %% Update Sensors
%     Sensor = updatesensor( state, stateDeriv );

    %Discrete Time recording @ 200 Hz
%     Hist = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control, PropState, Contact, localFlag, Sensor);
% end