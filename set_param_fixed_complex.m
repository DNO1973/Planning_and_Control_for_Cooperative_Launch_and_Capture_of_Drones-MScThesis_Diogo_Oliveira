
%**Paths with two straight trajectories**%

% initial uav conditions straight line
p0_line = [0;0;0];
psi0_line = pi/2 ;

gamma_limit_line = pi/6;
phi_limit_line = pi/6;
kphi_line = 10;

V_line = 9;





%reference straight line 1
c0_line1 = [0;0;0];
psi_l_line1 =pi/2;  %desired heading angle
gamma_l_line1 = 0; %desired flight path angle


%reference straight line 2
c0_line2 = [0;30;0];
psi_l_line2 =pi/2;  %desired heading angle
gamma_l_line2 = 0; %desired flight path angle

%reference straight line 3
c0_line3 = [0;700;0];
psi_l_line3 = pi/2;  %desired heading angle
gamma_l_line3 = 0; %desired flight path angle




% %reference straight line 2
% c0_line2 = [0;30;0];
% psi_l_line2 =0;  %desired heading angle
% gamma_l_line2 = 0; %desired flight path angle
% 
% %reference straight line 3
% c0_line3 = [30;30;0];
% psi_l_line3 = -pi/2;  %desired heading angle
% gamma_l_line3 = 0; %desired flight path angle





% put parameters into structure

%uav conditions
ParamFixComplex.p0 = p0_line;
ParamFixComplex.psi0 = psi0_line;

ParamFixComplex.gamma_limit = gamma_limit_line;
ParamFixComplex.phi_limit = phi_limit_line;
ParamFixComplex.kphi = kphi_line;

ParamFixComplex.V = V_line;





ParamFixComplex.c0_l = [c0_line1, c0_line2, c0_line3];
ParamFixComplex.psi_l = [psi_l_line1, psi_l_line2, psi_l_line3];
ParamFixComplex.gamma_l = [gamma_l_line1, gamma_l_line2, gamma_l_line3];

% %line one
% ParamFixComplex.c0_l1 = c0_line1;
% ParamFixComplex.psi_l1 = psi_l_line1;
% ParamFixComplex.gamma_l1 = gamma_l_line1;
% 
% 
% 
% %line two
% ParamFixComplex.c0_l2 = c0_line2;
% ParamFixComplex.psi_l2 = psi_l_line2;
% ParamFixComplex.gamma_l2 = gamma_l_line2;












%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initial conditions orbit
% 
%  Rh_orb = 30;
%  lambda_orb = 1;
%  gamma_h_orb = 15*pi/180;
%  psi_h_orb = pi/2;
%  
% k1_orb = 1;
% k2_orb = 10;
%  
% %center of helix
% ch_orb = [0; 0; 0];
% 
% p0_orb = [0.1;0;0];                                                         %start position on center of helix
% %p0_orb = ch_orb + [Rh_orb*cos(psi_h_orb) ; Rh_orb*sin(psi_h_orb) ; 0];     %start position on helix
% psi0_orb = pi/2;
% 
% gamma_limit_orb = pi/3;
% phi_limit_orb = pi/3;
% kphi_orb = 1;
% 
% V_orb = 9;
% %reference orbit
% 
% 
% 
% % put parameters into structure
% 
% ParamFixOrb.p0 = p0_orb;
% ParamFixOrb.psi0 = psi0_orb;
% ParamFixOrb.ch = ch_orb;
% 
% ParamFixOrb.gamma_limit = gamma_limit_orb;
% ParamFixOrb.phi_limit = phi_limit_orb;
% ParamFixOrb.kphi = kphi_orb;
% 
% ParamFixOrb.V = V_orb;
% 
% ParamFixOrb.Rh = Rh_orb;
% ParamFixOrb.lambda = lambda_orb;
% ParamFixOrb.gamma_h = gamma_h_orb;
% ParamFixOrb.psi_h = psi_h_orb;
% ParamFixOrb.k1 = k1_orb;
% ParamFixOrb.k2 = k2_orb;
%  





