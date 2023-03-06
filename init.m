% inicializations

kp=1; kv=5;

% Model and simulation parameters
Tend = 70;
dT = 0.1;
N = round(Tend/dT)+1;
g = 9.8;
m = 4;

% initial conditions
p0 = [0;0;0];
v0 = [0;0;0];
nx = 6;
nu = 3;
x = zeros(nx,N+1);
lbd = zeros(3,N+1);
u = zeros(nu,N);
x(:,1) = [p0;v0];
x_ref = [[0;0;0;0;0;0]*ones(1,1/dT+1),...
         [0.5;0;1;0;0;0]*ones(1,(Tend-1)/dT)];
t = 0:dT:Tend;

% put parameters into structure
Param.g = g;
Param.m = m;
Param.kp = kp;
Param.kv = kv;
Param.p0 = p0;
Param.v0 = v0;
%Param.R0 = eye(3);
%Param.om0 = [0;0;0.1];
Param.p_ref_static = [0.5;0;1];
Param.p_ref_static = [3;3;3];
