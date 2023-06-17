% initial conditions straight line
p0_line = [0;0;0];
psi0_line = 0 ;

gamma_limit_line = pi/6;
phi_limit_line = pi/6;
kphi_line = 10;

V_line = 9;

%reference straight line
c0_line = [50;20;20];
psi_l_line =0;  %desired heading angle
gamma_l_line = 0; %desired flight path angle


% put parameters into structure

ParamFixLine.p0 = p0_line;
ParamFixLine.psi0 = psi0_line;

ParamFixLine.gamma_limit = gamma_limit_line;
ParamFixLine.phi_limit = phi_limit_line;
ParamFixLine.kphi = kphi_line;

ParamFixLine.V = V_line;

ParamFixLine.c0 = c0_line;
ParamFixLine.psi_l = psi_l_line;
ParamFixLine.gamma_l = gamma_l_line;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initial conditions orbit

 Rh_orb = 50;
 lambda_orb = 1;
 gamma_h_orb = -0.1;% 15*pi/180;
 psi_h_orb = 0;
 
k1_orb = 1;
k2_orb = 10;
 
%center of helix
ch_orb = [10; 0; 0];

p0_orb = [0.1;0;0];                                                         %start position on center of helix
p0_orb = ch_orb + [Rh_orb*cos(psi_h_orb) ; Rh_orb*sin(psi_h_orb) ; 0];     %start position on helix
psi0_orb = pi/2;

gamma_limit_orb = pi/3;
phi_limit_orb = pi/3;
kphi_orb = 1;

V_orb = 9;
%reference orbit



% put parameters into structure

ParamFixOrb.p0 = p0_orb;
ParamFixOrb.psi0 = psi0_orb;
ParamFixOrb.ch = ch_orb;

ParamFixOrb.gamma_limit = gamma_limit_orb;
ParamFixOrb.phi_limit = phi_limit_orb;
ParamFixOrb.kphi = kphi_orb;

ParamFixOrb.V = V_orb;

ParamFixOrb.Rh = Rh_orb;
ParamFixOrb.lambda = lambda_orb;
ParamFixOrb.gamma_h = gamma_h_orb;
ParamFixOrb.psi_h = psi_h_orb;
ParamFixOrb.k1 = k1_orb;
ParamFixOrb.k2 = k2_orb;
 





