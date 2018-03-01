%sim_MonteCarlo.m Script to perform quadrotor Monte Carlo simulation

%   Description: Script to perform quadrotor Monte Carlo simulation.
%                Initial collision conditions are randomized between certain
%                ranges. Data is saved to the variable "Batch". Keep note
%                of the default data values, which are unchanged if the 
%                quadrotor does not actually collide with the wall.
%                (e.g., horizLoss = 9999, xVelRecovered = 9999, etc.)
%-------------------------------------------------------------------------%

Batch = [];
numTrials =20;

for iBatch = 1:numTrials 
    disp(iBatch)

    inclinationImpact = 20*rand+10; %degrees
    angle = (inclinationImpact - 0.0042477)/1.3836686;
    rollImpact = -angle; %degrees
    pitchImpact = -angle; %degrees
        %     rollImpact = 30*rand-15; %-15 to 15 deg
        %     pitchImpact = 90*rand-45; %-45 to 45 deg
    VxImpact =2*rand+0.5; %0.5 to 2.5 m/s
    yawImpact =45% 90*rand-45; %-45 to 45 deg
    
    [CrashData.ImpactIdentification,CrashData.FuzzyInfo,CrashData.Plot,CrashData.timeImpact] = startsim(VxImpact, rollImpact, pitchImpact, yawImpact);
    
    recoveryIdxs = [0;0;0]; %timeImpact, RS1 done, RS2 done
    
    Distfromwall = 9999;
    heightLoss = 9999;
    xVelRecovered = 9999;
    recoverySuccessful = 0;
    
    recoveryIdxs(1) = vlookup(CrashData.Plot.times,CrashData.timeImpact);
%     if ~isempty(find(CrashData.Plot.recoveryStage == 1,1)) %~ means the opposite, so ~isempty means if its not empty carry on
%         allIdxs = find(CrashData.Plot.recoveryStage == 1);
%         recoveryIdxs(2) = allIdxs(end);
%     end

        

     WallNormal=CrashData.ImpactIdentification.wallNormalWorld;
    % compute body z-axis direction, -1 because quad z-axis points down
for i=recoveryIdxs(1)+2:length(CrashData.Plot.times) %%how long before we start cehcking
    bodyFrameZAxis = quatrotate(CrashData.Plot.quaternions(1:4,i)', [0 0 -1]);
 
    if recoveryIdxs(2)==0
        if dot( WallNormal,bodyFrameZAxis) > 0
        recoveryIdxs(2)=1;
        recoveryIdxs(3)=CrashData.Plot.times(i);
        Distfromwall = sqrt(sum((CrashData.Plot.posns(1:2,i) - CrashData.Plot.posns(1:2, recoveryIdxs(1))).^2));
        heightLoss = CrashData.Plot.posns(3, recoveryIdxs(1)) - CrashData.Plot.posns(3, i);
        xVelRecovered =  CrashData.Plot.posnDerivs(1,i);
        end
    end
    
    theta = acos(dot(bodyFrameZAxis, WallNormal));
end
CrashData.recoverysucc=recoveryIdxs(2);
CrashData.recoverytimeaway= recoveryIdxs(3);
    CrashData.inclinationImpact=inclinationImpact;
    CrashData.theta =theta;
    CrashData.roll_atImpact = rollImpact;
    CrashData.pitch_atImpact = pitchImpact; %in degrees;
    CrashData.vel_atImpact = VxImpact ;
    CrashData.yaw_atImpact = yawImpact;
    CrashData.recoveryIdxs = recoveryIdxs;
    CrashData.DistanceFromWall = Distfromwall;
    CrashData.heightLoss = heightLoss;
    CrashData.xVelRecovered = xVelRecovered;
    CrashData.BodyZAxis=bodyFrameZAxis;
   
%     CrashData.recoverySuccessful = recoverySuccessful;
    Batch = [Batch;CrashData];
end

save('MonteCarlo1.mat')

