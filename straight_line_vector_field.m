function u  = straight_line_vector_field(p,  ParamFixLine)


%Variables

psi_l = ParamFixLine.psi_l;  %desired heading angle
gamma_l = ParamFixLine.gamma_l; %desired flight path angle

c_n = ParamFixLine.c0(1);    %path starting point
c_e = ParamFixLine.c0(2);
c_d = ParamFixLine.c0(3);

r_n = p(1,:);    %position
r_e = p(2,:);
r_d = p(3,:);
r = [r_n; r_e; r_d];

cl = [c_n; c_e; c_d];

ql = [cos(psi_l)*cos(gamma_l) ; sin(psi_l)*cos(gamma_l) ; -sin(gamma_l) ];
%direction of line




n_lon = [ -sin(psi_l) ; cos(psi_l) ; 0 ]; 

n_lat = [-cos(psi_l)*sin(gamma_l) ; -sin(psi_l)*sin(gamma_l) ; -cos(gamma_l) ];

k1 = 1;
k2 = 10;

%u_line = k1*(n_lon*transpose(n_lon) +  n_lat*transpose(n_lat))*(r - cl) + k2*cross(n_lon,n_lat);
u_line = - k1*(n_lon*n_lon' +  n_lat*n_lat')*(r - cl) - k2*cross(n_lon,n_lat);

u = ParamFixLine.V*(u_line./vecnorm(u_line))  ;




end
