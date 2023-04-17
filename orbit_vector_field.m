function u  = orbit_vector_field(p,  ParamFixOrb)




 Rh = ParamFixOrb.Rh;
 lambda = ParamFixOrb.lambda;
 gamma_h = ParamFixOrb.gamma_h;
 psi_h = ParamFixOrb.psi_h;
 
k1 = ParamFixOrb.k1;
k2 = ParamFixOrb.k2;
 

ch = ParamFixOrb.ch;

c_n = ch(1);    %center of helix
c_e = ch(2);
c_d = ch(3);


 r_n = p(1,:);    %position
r_e = p(2,:);
r_d = p(3,:);

 
%r0 = ch + [Rh*cos(psi_h) ; Rh*sin(psi_h) ; 0];

dalpha_cyl = ( [ 2*(r_n - c_n)/Rh ; 2*(r_e - c_e)/Rh ; zeros(1,1000) ] );

dalpha_pl = ( [ (tan(gamma_h)/lambda)*(-(r_e - c_e))./( (r_n - c_n).^2 + (r_e - c_e).^2 ) ; (tan(gamma_h)/lambda)*(r_n - c_n)./( (r_n - c_n).^2 + (r_e - c_e).^2 )  ; 1/Rh*ones(1,1000) ] );


alpha_cyl = ( (r_n - c_n)/Rh ).^2 + ( (r_e - c_e)/Rh ).^2 -1;

alpha_pl = ( (r_d - c_d)/Rh ).^2 + (tan(gamma_h)/lambda)*(atan( (r_e - c_e)./(r_n - c_n) ) - psi_h);

u_line = k1*(-alpha_cyl.*dalpha_cyl + alpha_pl.*dalpha_pl) + lambda*k2*( (2/Rh)*([ (r_e - c_e)/Rh ; -(r_n - c_n)/Rh ; lambda*tan(gamma_h)*ones(1,1000) ]) );

%u =  V*(u_line/norm(u_line));
u = ParamFixOrb.V*(u_line./vecnorm(u_line))  ;





end
