    
    %straight line
% figure(11);
% 
%  %plot3(out.fixlineout.Data(:,1), out.fixlineout.Data(:,2), out.fixlineout.Data(:,3), '--','Color','#D95319');
%     
% % distances = zeros(size(out.fixlineout.Data(:,1),1),1);
%  
% for i = 1:size(out.fixlineout.Data(:,1),1)
%     
%     dist = sqrt((h(4).XData - out.fixlineout.Data(i,1)).^2 + (h(4).YData - out.fixlineout.Data(i,2)).^2  + (h(4).ZData - out.fixlineout.Data(i,3)).^2);
%    % [~,idx] = min(dist,[],3); % index for the closest points, i.e. those with min distance.
%    distances(i) = min(dist);
% 
% end
% 
%     plot(out.fixorbout.Time,  distances, '-','Color','#0072BD');
%         hold on;
%       
%            grid on;
%   %  axis equal;
%    % axis([-50 50 -50 50 -50 50]);
% title('Tracking Error For Straight-Line Path');
%   %legend('Straight-Line Reference','Target Drone Trajectory','Reference Start Position','Target Drone Start Position');
%     xlabel('Time [s]');
%     ylabel('Error [m]');
%   
%      hold off;
%      
%      


%   
% %helical
% figure(11);
% 
%  %plot3(out.fixlineout.Data(:,1), out.fixlineout.Data(:,2), out.fixlineout.Data(:,3), '--','Color','#D95319');
%     
% % distances = zeros(size(out.fixlineout.Data(:,1),1),1);
%  
% 
% %h = findobj(gca,'Type','line');
% 
% for i = 1:size( out.fixorbout.Data(:,1),1)
%     
%     dist = sqrt((h(4).XData - out.fixorbout.Data(i,1)).^2 + (h(4).YData - out.fixorbout.Data(i,2)).^2  + (h(4).ZData - out.fixorbout.Data(i,3)).^2);
%    % [~,idx] = min(dist,[],3); % index for the closest points, i.e. those with min distance.
%    distances(i) = min(dist);
% 
% end
% 
%     plot(out.fixorbout.Time,  distances, '-','Color','#0072BD');
%         hold on;
%       
%            grid on;
%   %  axis equal;
%    % axis([-50 50 -50 50 -50 50]);
% title('Tracking Error For Helical Path');
%   %legend('Straight-Line Reference','Target Drone Trajectory','Reference Start Position','Target Drone Start Position');
%     xlabel('Time [s]');
%     ylabel('Error [m]');
%   
%      hold off;


% %square
% figure(11);
% 
%  %plot3(out.fixlineout.Data(:,1), out.fixlineout.Data(:,2), out.fixlineout.Data(:,3), '--','Color','#D95319');
%     
% % distances = zeros(size(out.fixlineout.Data(:,1),1),1);
%  
% 
% %h = findobj(gca,'Type','line');
% 
% for i = 1:size( out.fixcomplexout.Data(:,1),1)
%     
%      dist1 = sqrt((h(4 ).XData - out.fixcomplexout.Data(i,1)).^2 + (h(4).YData - out.fixcomplexout.Data(i,2)).^2  + (h(4).ZData - out.fixcomplexout.Data(i,3)).^2);
%    dist2 = sqrt((h( 5 ).XData - out.fixcomplexout.Data(i,1)).^2 + (h(5).YData - out.fixcomplexout.Data(i,2)).^2  + (h(5).ZData - out.fixcomplexout.Data(i,3)).^2);
%    dist3 = sqrt((h( 6 ).XData - out.fixcomplexout.Data(i,1)).^2 + (h(6).YData - out.fixcomplexout.Data(i,2)).^2  + (h(6).ZData - out.fixcomplexout.Data(i,3)).^2);
%    dist4 = sqrt((h( 7 ).XData - out.fixcomplexout.Data(i,1)).^2 + (h(7).YData - out.fixcomplexout.Data(i,2)).^2  + (h(7).ZData - out.fixcomplexout.Data(i,3)).^2);
%  distances(i) = min([dist1,dist2,dist3,dist4]);
% 
% end
% 
%     plot(out.fixcomplexout.Time,  distances, '-','Color','#0072BD');
%         hold on;
%       
%            grid on;
%   %  axis equal;
%    % axis([-50 50 -50 50 -50 50]);
% title('Tracking Error For Square Path');
%   %legend('Straight-Line Reference','Target Drone Trajectory','Reference Start Position','Target Drone Start Position');
%     xlabel('Time [s]');
%     ylabel('Error [m]');
%   
%      hold off;
% 


     
     %dubins
figure(11);


%h = findobj(gca,'Type','line');

for i = 1:size( out.fixcomplexout.Data(:,1),1)
    
    dist1 = sqrt((h(4 ).XData - out.fixcomplexout.Data(i,1)).^2 + (h(4).YData - out.fixcomplexout.Data(i,2)).^2  + (h(4).ZData - out.fixcomplexout.Data(i,3)).^2);
   dist2 = sqrt((h( 5 ).XData - out.fixcomplexout.Data(i,1)).^2 + (h(5).YData - out.fixcomplexout.Data(i,2)).^2  + (h(5).ZData - out.fixcomplexout.Data(i,3)).^2);
   dist3 = sqrt((h( 6 ).XData - out.fixcomplexout.Data(i,1)).^2 + (h(6).YData - out.fixcomplexout.Data(i,2)).^2  + (h(6).ZData - out.fixcomplexout.Data(i,3)).^2);
   dist4 = sqrt((h( 7 ).XData - out.fixcomplexout.Data(i,1)).^2 + (h(7).YData - out.fixcomplexout.Data(i,2)).^2  + (h(7).ZData - out.fixcomplexout.Data(i,3)).^2);
   dist5 = sqrt((h( 8 ).XData - out.fixcomplexout.Data(i,1)).^2 + (h(8).YData - out.fixcomplexout.Data(i,2)).^2  + (h(8).ZData - out.fixcomplexout.Data(i,3)).^2);
   dist6 = sqrt((h( 9 ).XData - out.fixcomplexout.Data(i,1)).^2 + (h(9).YData - out.fixcomplexout.Data(i,2)).^2  + (h(9).ZData - out.fixcomplexout.Data(i,3)).^2);
   
    
    
    
    
    distances(i) = min([dist1,dist2,dist3,dist4,dist5,dist6]);

end

    plot(out.fixcomplexout.Time,  distances, '-','Color','#0072BD');
        hold on;
      
           grid on;
  %  axis equal;
   % axis([-50 50 -50 50 -50 50]);
title('Tracking Error For Dubins Path');
  %legend('Straight-Line Reference','Target Drone Trajectory','Reference Start Position','Target Drone Start Position');
    xlabel('Time [s]');
    ylabel('Error [m]');
  
     hold off;
