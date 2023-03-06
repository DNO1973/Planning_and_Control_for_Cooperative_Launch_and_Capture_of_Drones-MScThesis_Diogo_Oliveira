
sstblue         = [0,128,255]/255;
sstlightblue    = [48,208,216]/255;
sstlighterblue  = [50,220,240]/255;
sstlightestblue = [60,230,255]/255;
sstgreen        = [43,191,92]/255;
sstlightgreen   = [140,255,200]/255;
sstlightergreen = [160,255,225]/255;
sstgray         = [70,70,70]/255;
sstlightgray    = [200,200,200]/255;

% 
% plot3(out.simout.Data(1,1), out.simout.Data(1,2), out.simout.Data(1,3),'o','Color',sstgray,'MarkerSize',8);
% 
% hold on;
% 
% plot3(out.simout.Data(end,1), out.simout.Data(end,2), out.simout.Data(end,3),'x','Color',sstgray,'MarkerSize',8);
% 
% 
% plot3(out.simout.Data(:,10), out.simout.Data(:,11), out.simout.Data(:,12), '-','Color',sstgreen);
% 
% plot3(out.simout.Data(:,1), out.simout.Data(:,2), out.simout.Data(:,3),'--','Color',sstblue);
% 
% grid on;
%   
% xlabel('x [m]');
% ylabel('y [m]');
% zlabel('z [m]');
% 
% legend('Start position', 'End position', 'Reference','Trajectory');
% 
% 
% hold off;

%%%%%%%%%


% psi_l = 3*pi/4;  
% gamma_l = pi;
% ql = [cos(psi_l)*cos(gamma_l) ; sin(psi_l)*cos(gamma_l) ; -sin(gamma_l) ];
% 
% 
%  
%  t = linspace(0,10000,100);
%  e = ql*t;
%  plot3( e(:,1), e(:,2),  e(:,3), '-');
%  
%  hold on;
% plot3(out.fixlineout.Data(:,1), out.fixlineout.Data(:,2), out.fixlineout.Data(:,3));
%  
%  grid on;
%   
% xlabel('x [m]');
% ylabel('y [m]');
% zlabel('z [m]');
% 
%  hold off;



%hold on;
plot3(out.fixorbout.Data(:,1), out.fixorbout.Data(:,2), out.fixorbout.Data(:,3));
 
 grid on;
  
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');

 hold off;