function [stateDeriv, Contact, PropState] = dynamicsystem(t,state,tStep,rpmControl,ImpactParams,rpmPrev,propCmds)

global g m I  Jr PROP_POSNS %Inertial and Geometric Parameters
global AERO_AREA AERO_DENS Cd Tv Kp Kq  %Aerodynamic Parameters
global timeImpact globalFlag %Simulation global variables
Kr= 5*10^-3;
Ixx = 0.01121976;
Iyy = 0.01122668;
Izz = 0.021082335;
Ixy = -5.62297e-05;
Iyz = -4.4954e-06;
Izx = -1.418e-08;  
Izzp=2.20751e-5;
    IzzT=Izz +Izzp;
IB = [Ixx Ixy Izx;Ixy Iyy Iyz;Izx Iyz Izz]; 
%% Save Global Variables locally
Kt_fiona = 8.7e-8; %used for thrust kt*rpm^2
Dt = 9.61e-10; %dt*rpm^2 used for torq coiff

%% State Initialization
q = [state(10);state(11);state(12);state(13)];
rotMat = quat2rotmat(q);

state = reshape(state,[max(size(state)),1]); %make sure state is column vector
stateDeriv = zeros(13,1); %initialize stateDeriv
PropState.rpm = rpmControl;
rpmDeriv=0;
% PropState.rpmDeriv = rpmDeriv;
%% Calculate External Forces and Moments
Fg = rotMat*[0;0;-m*g]; %Gravity 
V = 0;
% Fa = Tv*[-0.5*AERO_DENS*V^2*AERO_AREA*Cd;0;0]; %Aerodynamic
Fa=0;
% rpmControl
Ft = [0;0;-Kt_fiona*sum(rpmControl.^2)];%Thruster
Mx = -Kt_fiona*PROP_POSNS(2,:)*(rpmControl.^2)-state(5)*Jr*sum(rpm2rad(rpmControl));
My = Kt_fiona*PROP_POSNS(1,:)*(rpmControl.^2)+state(4)*Jr*sum(rpm2rad(rpmControl));
Mz =  [-Dt Dt -Dt Dt]*(rpmControl.^2)-Kr*state(6);%% reverse signs of Dt %%mz should be equal and oppsite

%% Update State Derivative
% stateDeriv(1:3) = (Fg + Fa + Ft + totalContactForce - m*cross(state(4:6),state(1:3)))/m;
stateDeriv(1:3) = (Fg +  Ft  - m*cross(state(4:6),state(1:3)))/m;
stateDeriv(4:6) = inv(IB)*([Mx;My;Mz]-cross(state(4:6),IB*state(4:6)));
stateDeriv(7:9) = rotMat'*state(1:3);
stateDeriv(10:13) = -0.5*quatmultiply([0;state(4:6)],q);


end

