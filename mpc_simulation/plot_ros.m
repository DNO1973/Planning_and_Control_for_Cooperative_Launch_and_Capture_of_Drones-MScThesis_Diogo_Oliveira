
clear all; clc; close all;


states = readmatrix('state_logs.csv')


 figure(1);
plot3(states(:,8),states(:,9),states(:,10));

hold on;

plot3(states(:,1),states(:,2),states(:,3), '--');

title('Target and Shuttle Positions');
xlabel('X[m]'); ylabel('y[m]'); zlabel('z[m]');
set(gca, 'Zdir', 'reverse');
set(gca, 'Ydir', 'reverse');
%set(gca, 'Xdir', 'reverse');
legend('Target', 'Shuttle');
grid on;
axis equal;
%axis([-50 500 -50 100 -50 50]);
  
hold off;



     figure(2);
  
     plot( states(:,8:10), '-');
     hold on;
        plot( states(:,1:3), '--');
     title('Shuttle and Target Positions');

     legend('x_{target}','y_{target}','z_{target}','x_{shuttle}','y_{shuttle}','z_{shuttle}','Interpreter','tex');  
    
 hold off;

