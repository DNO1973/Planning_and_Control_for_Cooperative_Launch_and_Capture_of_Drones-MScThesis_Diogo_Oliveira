function u = drone_controller(p,v,p_ref,P)
%DRONE_CONTROLLER Summary of this function goes here
%   Detailed explanation goes here
    
    u = -diag([P.kp/2,P.kp/2,P.kp])*(p-p_ref) + ...
        -diag([P.kv/2,P.kv/2,P.kv])*v + [0;0;P.m*P.g];
    
end

