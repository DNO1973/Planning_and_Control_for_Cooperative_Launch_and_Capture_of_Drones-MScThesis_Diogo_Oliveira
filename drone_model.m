function [dp,dv,dpsi] = drone_model(p,v,psi,u,P)
%DRONE_MODEL Summary of this function goes here
%   Detailed explanation goes here

%     p = x(1:3);
%     v = x(4:6);
%     om =x(7:9);
%     R = reshape(x(10:18),3,3);

    dp = v;
    dv = -[0;0;P.g] + 1/P.m*u;
    dpsi = 0; % zeros(3,1);
 
    
end

