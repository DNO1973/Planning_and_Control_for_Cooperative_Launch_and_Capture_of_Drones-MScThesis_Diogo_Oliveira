
sstblue         = [0,128,255]/255;
sstlightblue    = [48,208,216]/255;
sstlighterblue  = [50,220,240]/255;
sstlightestblue = [60,230,255]/255;
sstgreen        = [43,191,92]/255;
sstlightgreen   = [140,255,200]/255;
sstlightergreen = [160,255,225]/255;
sstgray         = [70,70,70]/255;
sstlightgray    = [200,200,200]/255;




    


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

auxPoint=1; %apagar isto depois de corrigir orbitais com gamma!=0 consecutivas
    
number_of_segments = size(ParamFixComplex.paths);
number_of_segments = number_of_segments(2);

 figure(1);
 t = out.fixcomplexout.Time';

    for segment = 1:number_of_segments
        
        if ParamFixComplex.paths(segment) == 0
            
            
            if ~(segment == number_of_segments)
                psi_l = ParamFixComplex.psi_l(segment); 
                gamma_l = ParamFixComplex.gamma_l(segment);
                c_n = ParamFixComplex.c0(1,segment);    
                 c_e = ParamFixComplex.c0(2,segment);
                 c_d = ParamFixComplex.c0(3,segment);
                 
                 c_n2 = ParamFixComplex.c0(1,segment + 1);    
                 c_e2 = ParamFixComplex.c0(2,segment + 1);
                 c_d2 = ParamFixComplex.c0(3,segment + 1);
                cl = [c_n ; c_e; c_d];
                ql = [cos(psi_l)*cos(gamma_l) ; sin(psi_l)*cos(gamma_l) ; -sin(gamma_l) ];

                
                 distance_to_next_path = sqrt((c_n - c_n2)^2 + (c_e - c_e2)^2 + (c_d - c_d2)^2);


                 path_lenght = 1:distance_to_next_path;
                 r =cl+ ql*path_lenght;
                 % r =cl+ ql*1000;

                 plot3( r(1,:), r(2,:),  r(3,:), '--','Color','r');

                 hold on;
            
            else
                 psi_l = ParamFixComplex.psi_l(segment); 
                gamma_l = ParamFixComplex.gamma_l(segment);
                c_n = ParamFixComplex.c0(1,segment);    
                 c_e = ParamFixComplex.c0(2,segment);
                 c_d = ParamFixComplex.c0(3,segment);
                cl = [c_n ; c_e; c_d];
                ql = [cos(psi_l)*cos(gamma_l) ; sin(psi_l)*cos(gamma_l) ; -sin(gamma_l) ];



                 %r =cl+ ql*t;
                  path_lenght = 1:500;
                  r =cl+ ql*path_lenght;

                 plot3( r(1,:), r(2,:),  r(3,:), '--','Color','r');

                 hold on;
             
            end
            
        else  
            if ~(segment == number_of_segments)
                Rh = ParamFixComplex.Rh(segment);
                 lambda = ParamFixComplex.lambda(segment);
                 gamma_h = ParamFixComplex.gamma_h(segment);
                 psi_h = ParamFixComplex.psi_h(segment);

                c_n = ParamFixComplex.c0(1,segment);   
                c_e = ParamFixComplex.c0(2,segment);
                c_d = ParamFixComplex.c0(3,segment);
                 ch = [c_n ; c_e; c_d];
                    
                  c_n2 = ParamFixComplex.c0(1,segment + 1);    
                 c_e2 = ParamFixComplex.c0(2,segment + 1);
                 c_d2 = ParamFixComplex.c0(3,segment + 1);
                  ch2 = [c_n2 ; c_e2; c_d2];
                    
                  
%                    path_lenght = 1:0.01:10;
%                  r = ch + [Rh*cos(lambda*path_lenght + psi_h) ; Rh*sin(lambda*path_lenght + psi_h) ; -path_lenght*Rh*tan(gamma_h)];
% 
% 
%                 plot3(r(1,:), r(2,:), r(3,:), '--','Color','r'); 
% 
%                 hold on;
                  
%                   path_lenght = 1:0.01:10;
%                   r = ch + [Rh*cos(lambda*path_lenght + psi_h) ; Rh*sin(lambda*path_lenght + psi_h) ; -path_lenght*Rh*tan(gamma_h)];
%                   n = size(path_lenght,2);
%                 
%                  for point = 1:n
%                      distance_to_next_path = sqrt((r(1,point) - c_n2)^2 + (r(2,point) - c_e2)^2 + (r(3,point) - c_d2)^2);
%                      aux = 'r';
%                     % plot3(r(1,point), r(2,point), r(3,point), 'o','Color',aux); 
%                     
%                      fprintf('distance %f  r %f %f %f  ch %d %d %d \n' , distance_to_next_path, r(1,point),r(2,point),r(3,point), c_n2, c_e2, c_d2);
%                         hold on;
%                      if distance_to_next_path <= 1
%                            % plot3(r(1,point), r(2,point), r(3,point), 'o','Color',aux);
%                            auxPoint = point;
%                             break;
%    
%                      end
%                     % plot3(r(1,point), r(2,point), r(3,point), 'o','Color',aux);
%                      
%                  end
%                  
%                 
%                  disp(auxPoint);
%                  for point = 1:n
%                      if point >= auxPoint
%                          r(1,point) = 0;
%                         r(2,point) = 0;
%                         r(3,point) = 0;
%                      end
%                      
%                  end
%                   plot3(r(1,:), r(2,:), r(3,:), '--','Color','r');
                 

                path_lenght = 1:0.01:10;
                  r = ch + [Rh*cos(lambda*path_lenght + psi_h) ; Rh*sin(lambda*path_lenght + psi_h) ; -path_lenght*Rh*tan(gamma_h)];
                  n = size(path_lenght,2);
                
                 for point = 1:n
                     distance_to_next_path = sqrt((r(1,point) - c_n2)^2 + (r(2,point) - c_e2)^2 + (r(3,point) - c_d2)^2);
                    if distance_to_next_path <= 1
                       auxPoint = point;
                            break;
                     end                     
                 end
                 
                 rf = r(:,1:auxPoint);
                   
                  plot3(rf(1,:), rf(2,:), rf(3,:), '--','Color','r');
                    hold on;
                  %plot3(rf(1,1), rf(2,1), rf(3,1),'o','Color',sstgreen,'MarkerSize',10);
                 % plot3(rf(1,end), rf(2,end), rf(3,end),'x','Color',sstgray,'MarkerSize',10);
                 
            else
            
            
                 Rh = ParamFixComplex.Rh(segment);
                 lambda = ParamFixComplex.lambda(segment);
                 gamma_h = ParamFixComplex.gamma_h(segment);
                 psi_h = ParamFixComplex.psi_h(segment);

                c_n = ParamFixComplex.c0(1,segment);   
                c_e = ParamFixComplex.c0(2,segment);
                c_d = ParamFixComplex.c0(3,segment);
                  ch = [c_n ; c_e; c_d];


                  path_lenght = 1:0.01:10;
                 r = ch + [Rh*cos(lambda*path_lenght + psi_h) ; Rh*sin(lambda*path_lenght + psi_h) ; -path_lenght*Rh*tan(gamma_h)];


                plot3(r(1,:), r(2,:), r(3,:), '--','Color','r'); 

                hold on;
            end
        end
    end


    plot3(out.fixcomplexout.Data(:,1), out.fixcomplexout.Data(:,2), out.fixcomplexout.Data(:,3),'Color',sstblue);
    hold on; 
   % plot3(r(1,1), r(2,1), r(3,1),'o','Color',sstgreen,'MarkerSize',10);
   % plot3(r(1,end), r(2,end), r(3,end),'x','Color',sstgray,'MarkerSize',10);
    
    plot3(out.fixcomplexout.Data(1,1), out.fixcomplexout.Data(1,2), out.fixcomplexout.Data(1,3),'o','Color',sstblue,'MarkerSize',8);
    plot3(out.fixcomplexout.Data(end,1), out.fixcomplexout.Data(end,2), out.fixcomplexout.Data(end,3),'x','Color',sstblue,'MarkerSize',8);

    

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






 