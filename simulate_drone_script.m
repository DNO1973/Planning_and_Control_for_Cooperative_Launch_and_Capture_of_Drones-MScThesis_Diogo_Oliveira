% NOVA School of Science and Technology
% Department of Electrical and Computer Engineering
% IEEC course, fall 2021
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% Summary: simulate simple dynamic system model of a drone

init;

for k = 1:N,
    
    % get state vector and plot it
    p = x(1:3,k);
    v = x(4:6,k);
    p_ref = x_ref(1:3,k);
    %p_ref = Param.p_ref_static(1:3,k);
    p_ref = [1;1;1];
  
  
    
    if k > 200
        p_ref = [1.5;2;1];
        
    end
    
     if k > 400
        p_ref = [0;2;0.5];
        
    end
    

    % nonlinear controller
    u(:,k) = drone_controller(p,v,p_ref,Param);
    
    % simplified nonlinear model of a drone
    [dot_p,dot_v] = drone_model(p,v,[],u(:,k),Param);
    dot_x = [dot_p;dot_v];
    
    % discretized system using forward Euler integration
    x(:,k+1) = x(:,k) + dT*dot_x;
    
    % auxiliary drone attitude computation from input
   % f_des = 1/m*u(:,k);
   % zB = f_des/norm(f_des);
    %xC = [1;0;0];
   % yB = cross(zB,xC);
    %xB = cross(yB,zB);
   % R = [xB,yB,zB];
   % lbd_full(:,k) = R2Euler(R);
    
end
% auxiliary drone final attitude computation
%lbd_full(:,k+1) = lbd_full(:,k);

% show results plot
show_data;
