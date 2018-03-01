function accelref = calculaterefacceleration(outputFLP, wallNormalWorld)
%calculaterefacceleration.m Calculate a_ref for Recovery Controller
%   Author: Gareth Dicker (gareth.dicker@mail.mcgill.ca)
%   Last Updated: 
%   Description: 
%-------------------------------------------------------------------------%
accelref = 9.81*outputFLP*wallNormalWorld; %9.81 is the ka value. 

if outputFLP < 0   % reduce into the wall setpoint by half if away from wall
    accelref = [0;0;0];
end

end