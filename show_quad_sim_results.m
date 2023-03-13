
sstblue         = [0,128,255]/255;
sstlightblue    = [48,208,216]/255;
sstlighterblue  = [50,220,240]/255;
sstlightestblue = [60,230,255]/255;
sstgreen        = [43,191,92]/255;
sstlightgreen   = [140,255,200]/255;
sstlightergreen = [160,255,225]/255;
sstgray         = [70,70,70]/255;
sstlightgray    = [200,200,200]/255;


plot3(out.quadout.Data(1,1), out.quadout.Data(1,2), out.quadout.Data(1,3),'o','Color',sstgray,'MarkerSize',8);

hold on;

plot3(out.quadout.Data(end,1), out.quadout.Data(end,2), out.quadout.Data(end,3),'x','Color',sstgray,'MarkerSize',8);


plot3(out.quadout.Data(:,10), out.quadout.Data(:,11), out.quadout.Data(:,12), '-','Color',sstgreen);

plot3(out.quadout.Data(:,1), out.quadout.Data(:,2), out.quadout.Data(:,3),'--','Color',sstblue);

grid on;
  
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');

legend('Start position', 'End position', 'Reference','Trajectory');


hold off;

