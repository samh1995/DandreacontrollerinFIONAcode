function [Control] = controllerfailrecover(tStep, Pose, Twist, Control)
% Performs collision recovery control.
%
% Inputs: 
%   dt              :   time step (default 1/200 sec)
%   Pose            :   Struct of the pose of the quadrotor
%   Twist           :   Struct of the twist of the quadrotor
%   Control         :   Struct of the control variables and output
%   Hist.control    :   History of control signals
%
% Outputs:
%   Control         :   Struct of the control variables and output
global g  m 
%Fiona code
Kt_fiona = 8.699950983e-8; 
Dt = 9.609945846e-10;


Kf = 7.933*10^-6; 
Kt = 0.011045977; 

Ixx = 0.01121976;
Iyy = 0.01122668;
Izz = 0.021082335;
Ixy = -5.62297e-05;
Iyz = -4.4954e-06;
Izx = -1.418e-08;  
Izzp=2.20751e-5;
    IzzT=Izz +Izzp;
IB = [Ixx Ixy Izx;Ixy Iyy Iyz;Izx Iyz Izz]; 
IT = [Ixx Ixy Izx;Ixy Iyy Iyz;Izx Iyz IzzT]; 


wbar1= 7032.028;
wbar3= 7032.028;
wbar2=4972.33727;
fbar1=  4.302075109586292;
fbar2= 2.151045166432399;
% fbar=[(wbar1)^2*Kt_fiona; (wbar2)^2*Kt_fiona; (wbar1)^2*Kt_fiona; 0];
wbbar=[0;1.2861838646001;25.8863392279462];
wbbar3=25.8863392279462;
nbar=[0;0.0496243584044;0.9987682823025];
l=0.1807;

%% Rotation from Fiona to dandrea coordinates
R_F2D=[ sqrt(2)/2    sqrt(2)/2            0;
        sqrt(2)/2     -sqrt(2)/2          0;
        0               0                -1];
quat = [Pose.attQuat(1);Pose.attQuat(2);Pose.attQuat(3);Pose.attQuat(4)]/norm(Pose.attQuat(1:4));
R_I2F = quat2rotmat(quat); %from inertial to fiona
R_I2D=R_F2D*R_I2F;
R_D2I=R_I2D';
q_DandreaInitial=rotmat2quat(R_D2I)
%% LQR controller: solving for k 
a=(IT(1,1)-IT(3,3))*wbbar3/IB(1,1)-Izzp*(736.411+520.7221331489+736.411)/IB(1,1)
A=[0    a    0    0 ;
   -a   0    0    0 ;
   0  -nbar(3) 0 wbbar(3);
   nbar(3) 0 -wbbar(3) 0;]
B=l/IB(1,1)*[0 1;1 0; 0 0; 0 0]
C=[0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0];
D=[0 0;0 0 ;0 0 ;0 0 ];
q=[1 1 20 20];
Q=diag(q);
R=[ 1 0; 0 1];
Y=0;
N=0;                   
[K,S,e]=lqr(A,B,Q,R,N);
K
%% Solving for control inputs
% % Euler=R_F2D*Pose.attEuler(1:3);
% % q_dand = angle2quat(Euler(3),Euler(2),Euler(1),'ZYX'); %find dand quat using the euler found from fionas quat
% % R_dand=quat2rotmat(q_dand);%% from dandrea body to inertial frame
f_total=norm(m/nbar(3)*(Control.acc-[0;0;-g])) %ftotal in world frame

n_desired=m/nbar(3)*R_I2D*(Control.acc-[0;0;-g])/norm(m/nbar(3)*(Control.acc-[0;0;-g]))
% n_desired=m/nbar(3)*quat2rotmat(Pose.attQuat)*R_F2D*(Control.desiredaccel-[0;0;-g])/norm(m/nbar(3)*(Control.desiredaccel-[0;0;-g]));%ndeisred in body frame. quat2rotmat converts from inertial to body it was invR
pqrD=R_F2D*[Twist.angVel(1);Twist.angVel(2);Twist.angVel(3)]
s=[pqrD(1)-wbbar(1);pqrD(2)-wbbar(2);n_desired(1)-nbar(1);n_desired(2)-nbar(2)]
u=-K*s

  
%% solving for forces convert to rpm
  G=[1 1 1; -1 0 1; 0 1 0];
  H=[(f_total); u(1)-fbar1+fbar1;u(2)+fbar2];
  f=linsolve(G,H)
  f(4)=0;
 
% Kfiona = 8.7e-8;
  rpm=sqrt(abs(f)/Kt_fiona)

%  rpm1=f(1)/Kf
%  rpm2=f(2)/Kf
%  rpm3=f(3)/Kf
%  rpm4=f(4)/Kf
  Control.rpm=rpm;
end