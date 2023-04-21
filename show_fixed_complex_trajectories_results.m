
sstblue         = [0,128,255]/255;
sstlightblue    = [48,208,216]/255;
sstlighterblue  = [50,220,240]/255;
sstlightestblue = [60,230,255]/255;
sstgreen        = [43,191,92]/255;
sstlightgreen   = [140,255,200]/255;
sstlightergreen = [160,255,225]/255;
sstgray         = [70,70,70]/255;
sstlightgray    = [200,200,200]/255;



display =0; %

%o display das referencias ainda nao esta a funcionar muito bem

if display == 0 %straight
    


%     psi_l = ParamFixComplex.psi_l1;  
%     gamma_l = ParamFixComplex.gamma_l1;
%     cl = ParamFixComplex.c0_l1;
%     ql = [cos(psi_l)*cos(gamma_l) ; sin(psi_l)*cos(gamma_l) ; -sin(gamma_l) ];
% 
% 
% 
%     figure(1);
% 
%      t = out.fixcomplexout.Time';
% 
%      r =cl+ ql*t;
% 
%   
%      plot3( r(1,:), r(2,:),  r(3,:), '.');
% 
%      hold on;
%      
%       psi_l = ParamFixComplex.psi_l2;  
%     gamma_l = ParamFixComplex.gamma_l2;
%     cl = ParamFixComplex.c0_l2;
%     ql = [cos(psi_l)*cos(gamma_l) ; sin(psi_l)*cos(gamma_l) ; -sin(gamma_l) ];
%     
%     r =cl+ ql*t;
% 
%   
%      plot3( r(1,:), r(2,:),  r(3,:), '.');


    
number_of_segments = size(ParamFixComplex.paths);
number_of_segments = number_of_segments(2);

 figure(1);
 t = out.fixcomplexout.Time';

    for segment = 1:number_of_segments
        
        if ParamFixComplex.paths(segment) == 0
            
             psi_l = ParamFixComplex.psi_l(segment); 
            gamma_l = ParamFixComplex.gamma_l(segment);
            c_n = ParamFixComplex.c0(1,segment);    
             c_e = ParamFixComplex.c0(2,segment);
             c_d = ParamFixComplex.c0(3,segment);
            cl = [c_n ; c_e; c_d];
            ql = [cos(psi_l)*cos(gamma_l) ; sin(psi_l)*cos(gamma_l) ; -sin(gamma_l) ];
        
        
        
             r =cl+ ql*t;
        
          
             plot3( r(1,:), r(2,:),  r(3,:), '-','Color',sstgray);
        
             hold on;
            
        else   
             Rh = ParamFixComplex.Rh(segment);
             lambda = ParamFixComplex.lambda(segment);
             gamma_h = ParamFixComplex.gamma_h(segment);
             psi_h = ParamFixComplex.psi_h(segment);

            c_n = ParamFixComplex.c0(1,segment);   
            c_e = ParamFixComplex.c0(2,segment);
            c_d = ParamFixComplex.c0(3,segment);
              ch = [c_n ; c_e; c_d];
            

       

            r = [Rh*cos(lambda*t + psi_h) ; Rh*sin(lambda*t + psi_h) ; -t*Rh*tan(gamma_h)];

            %plot3(r(1,:), r(2,:), r(3,:), '-','Color',sstgray); 

            hold on;
            
        end
    end


    plot3(out.fixcomplexout.Data(:,1), out.fixcomplexout.Data(:,2), out.fixcomplexout.Data(:,3));
    hold on; 
   % plot3(r(1,1), r(2,1), r(3,1),'o','Color',sstgreen,'MarkerSize',10);
   % plot3(r(1,end), r(2,end), r(3,end),'x','Color',sstgray,'MarkerSize',10);
    
    plot3(out.fixcomplexout.Data(1,1), out.fixcomplexout.Data(1,2), out.fixcomplexout.Data(1,3),'o','Color',sstgray,'MarkerSize',8);
    plot3(out.fixcomplexout.Data(end,1), out.fixcomplexout.Data(end,2), out.fixcomplexout.Data(end,3),'x','Color',sstgray,'MarkerSize',8);

    

     grid on;
    axis equal;
    axis([-50 500 -50 100 -50 500]);
  
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');

   
    
    hold off;
    
%     
%     
%     figB = axis;
%     nsteps = 10;
%     steps = [(figB(2)-figB(1))/(nsteps-1);(figB(4)-figB(3))/(nsteps-1);(figB(6)-figB(5))/(nsteps-1)];
%     [X,Y,Z] = meshgrid(figB(1):steps(1):figB(2),figB(3):steps(2):figB(4),figB(5):steps(3):figB(6));
%     P = [   reshape(X,1,[])
%             reshape(Y,1,[])
%             reshape(Z,1,[]) ];
%     Vraw = straight_line_vector_field(P,ParamFixLine);
%     U = reshape(Vraw(1,:),nsteps,nsteps,nsteps);
%     V = reshape(Vraw(2,:),nsteps,nsteps,nsteps);
%     W = reshape(Vraw(3,:),nsteps,nsteps,nsteps)*0;
%     %quiver3(X,Y,Z,U,V,W);
%     idx_plane = (Z==0);
%     XX = X(idx_plane);
%     YY = Y(idx_plane);
%     UU = U(idx_plane);
%     VV = V(idx_plane);
%     %quiver(X,Y,U,V);
%    
%     
%     
%     
%     
%     
%     
%     
%      hold off;

% 
%      figure(2);
%      plot(t,r);
%      hold on;
%      legend('x','y','z');
%      plot(out.fixlineout.Time, out.fixlineout.Data(:,1:3));
%      legend('x','y','z','x1','y2','z3');
% %      hold off;
%      
%       figure(3);
%       
%      plot(t,r(1,:));
%      title('north');
%      hold on;
%      legend('x','x1');
%      plot(out.fixlineout.Time, out.fixlineout.Data(:,1));
%      hold off;
% 
%       figure(4);
%       
%      plot(t,r(2,:));
%      title('east');
%      hold on;
%      legend('y','y1');
%      plot(out.fixlineout.Time, out.fixlineout.Data(:,2));
%      hold off;
% 
%       figure(5);
%       
%      plot(t,r(3,:));
%      title('down');
%      hold on;
%      legend('z','z1');
%      plot(out.fixlineout.Time, out.fixlineout.Data(:,3));
%      hold off;







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

    
    
    
    
    
    
    
    
    
    figB = axis;
    nsteps = 10;
    steps = [(figB(2)-figB(1))/(nsteps-1);(figB(4)-figB(3))/(nsteps-1);(figB(6)-figB(5))/(nsteps-1)];
    [X,Y,Z] = meshgrid(figB(1):steps(1):figB(2),figB(3):steps(2):figB(4),figB(5):steps(3):figB(6));
    P = [   reshape(X,1,[])
            reshape(Y,1,[])
            reshape(Z,1,[]) ];
    Vraw = orbit_vector_field(P,ParamFixOrb);
    U = reshape(Vraw(1,:),nsteps,nsteps,nsteps);
    V = reshape(Vraw(2,:),nsteps,nsteps,nsteps);
    W = reshape(Vraw(3,:),nsteps,nsteps,nsteps);
    %quiver3(X,Y,Z,U,V,W);
    idx_plane = (Z==0);
    XX = X(idx_plane);
    YY = Y(idx_plane);
    UU = U(idx_plane);
    VV = V(idx_plane);
    quiver(X,Y,U,V);
   
    
    
    
    
    
    
    
    
    
    
    
    
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
     
     
      figure(6);
      
     %plot(t,-r(3,:));  %rever referenciais
     title('psi');
     %hold on;
     legend('z','z1');
     plot(out.fixorbout.Time, out.fixorbout.Data(:,4));
     hold on;
     plot(out.fixorbout.Time, out.fixorbout.Data(:,7));
     hold off;
     
     
      figure(7);
      
     %plot(t,-r(3,:));  %rever referenciais
     title('gamma');
     %hold on;
     legend('z','z1');
     plot(out.fixorbout.Time, out.fixorbout.Data(:,5));
     hold off;
     
      figure(8);
      
     %plot(t,-r(3,:));  %rever referenciais
     title('phi');
     %hold on;
     legend('z','z1');
     plot(out.fixorbout.Time, out.fixorbout.Data(:,6));
     hold off;
end

 