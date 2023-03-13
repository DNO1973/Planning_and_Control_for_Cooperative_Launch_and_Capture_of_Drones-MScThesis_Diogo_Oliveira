
sstblue         = [0,128,255]/255;
sstlightblue    = [48,208,216]/255;
sstlighterblue  = [50,220,240]/255;
sstlightestblue = [60,230,255]/255;
sstgreen        = [43,191,92]/255;
sstlightgreen   = [140,255,200]/255;
sstlightergreen = [160,255,225]/255;
sstgray         = [70,70,70]/255;
sstlightgray    = [200,200,200]/255;



display =0; % 0 - straight line , 1 - orbit



if display == 1 %straight
    


    psi_l = ParamFixLine.psi_l;  
    gamma_l = ParamFixLine.gamma_l;
    cl = ParamFixLine.c0;
    ql = [cos(psi_l)*cos(gamma_l) ; sin(psi_l)*cos(gamma_l) ; -sin(gamma_l) ];



    figure(1);

     t = out.fixorbout.Time';

     r =cl+ ql*t;

  
     plot3( r(1,:), r(2,:),  r(3,:), '.');

     hold on;
    plot3(out.fixlineout.Data(:,1), out.fixlineout.Data(:,2), out.fixlineout.Data(:,3));
    
    plot3(r(1,1), r(2,1), r(3,1),'o','Color',sstgreen,'MarkerSize',10);
    plot3(r(1,end), r(2,end), r(3,end),'x','Color',sstgray,'MarkerSize',10);
    
    plot3(out.fixlineout.Data(1,1), out.fixlineout.Data(1,2), out.fixlineout.Data(1,3),'o','Color',sstgray,'MarkerSize',8);
    plot3(out.fixlineout.Data(end,1), out.fixlineout.Data(end,2), out.fixlineout.Data(end,3),'x','Color',sstgray,'MarkerSize',8);

    

     grid on;

    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');

     hold off;

% 
%      figure(2);
%      plot(t,r);
%      hold on;
%      legend('x','y','z');
%      plot(out.fixlineout.Time, out.fixlineout.Data(:,1:3));
%      legend('x','y','z','x1','y2','z3');
%      hold off;
     
      figure(3);
      
     plot(t,r(1,:));
     title('north');
     hold on;
     legend('x','x1');
     plot(out.fixlineout.Time, out.fixlineout.Data(:,1));
     hold off;

      figure(4);
      
     plot(t,r(2,:));
     title('east');
     hold on;
     legend('y','y1');
     plot(out.fixlineout.Time, out.fixlineout.Data(:,2));
     hold off;

      figure(5);
      
     plot(t,r(3,:));
     title('down');
     hold on;
     legend('z','z1');
     plot(out.fixlineout.Time, out.fixlineout.Data(:,3));
     hold off;







else %orbit
    
    
    Rh = ParamFixOrb.Rh;
    lambda = ParamFixOrb.lambda;
     gamma_h = ParamFixOrb.gamma_h;
     psi_h = ParamFixOrb.psi_h;
     ch = ParamFixOrb.ch;


    t = out.fixorbout.Time';

    r = [Rh*cos(lambda*t + psi_h) ; Rh*sin(lambda*t + psi_h) ; -t*Rh*tan(gamma_h)];

    plot3(r(1,:), r(2,:), r(3,:), '--'); 

    hold on;
    
    
    
    plot3(r(1,1), r(2,1), r(3,1),'o','Color',sstgreen,'MarkerSize',10);
    plot3(r(1,end), r(2,end), r(3,end),'x','Color',sstgray,'MarkerSize',10);
    
    plot3(out.fixorbout.Data(1,1), out.fixorbout.Data(1,2), out.fixorbout.Data(1,3),'o','Color',sstgray,'MarkerSize',8);
    plot3(out.fixorbout.Data(end,1), out.fixorbout.Data(end,2), out.fixorbout.Data(end,3),'x','Color',sstgray,'MarkerSize',8);

    
    
    
    plot3(out.fixorbout.Data(:,1), out.fixorbout.Data(:,2), out.fixorbout.Data(:,3));

     grid on;

    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');

     hold off;
     
     
     
     
     
     
     
      figure(3);
      
     plot(t,r(1,:));
     title('north');
     hold on;
     legend('x','x1');
     plot(out.fixorbout.Time, out.fixorbout.Data(:,1));
     hold off;

      figure(4);
      
     plot(t,r(2,:));
     title('east');
     hold on;
     legend('y','y1');
     plot(out.fixorbout.Time, out.fixorbout.Data(:,2));
     hold off;

      figure(5);
      
     plot(t,-r(3,:));  %rever referenciais
     title('down');
     hold on;
     legend('z','z1');
     plot(out.fixorbout.Time, out.fixorbout.Data(:,3));
     hold off;
     
     
end

 