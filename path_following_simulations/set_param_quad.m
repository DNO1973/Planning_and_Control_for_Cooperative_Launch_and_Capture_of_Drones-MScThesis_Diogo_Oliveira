% inicializations

%kp=5; kv=10;
kp=10; kv=25;

m = 5; %mass

% initial conditions
p0 = [-50;25;-40];
v0 = [0;0;0];

% put parameters into structure

ParamQuad.m = m;
ParamQuad.kp = kp;
ParamQuad.kv = kv;
ParamQuad.p0 = p0;
ParamQuad.v0 = v0;
