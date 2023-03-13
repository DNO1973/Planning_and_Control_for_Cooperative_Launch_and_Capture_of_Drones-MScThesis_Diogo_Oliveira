
sstblue         = [0,128,255]/255;
sstlightblue    = [48,208,216]/255;
sstlighterblue  = [50,220,240]/255;
sstlightestblue = [60,230,255]/255;
sstgreen        = [43,191,92]/255;
sstlightgreen   = [140,255,200]/255;
sstlightergreen = [160,255,225]/255;
sstgray         = [70,70,70]/255;
sstlightgray    = [200,200,200]/255;

  
   
plot3(out.fixquadout.Data(:,1), out.fixquadout.Data(:,2), out.fixquadout.Data(:,3),'-','Color',sstblue);
hold on;


plot3(out.fixquadout.Data(:,4), out.fixquadout.Data(:,5), out.fixquadout.Data(:,6),'-','Color',sstgreen);



plot3(out.fixquadout.Data(1,1), out.fixquadout.Data(1,2), out.fixquadout.Data(1,3),'o','Color',sstgray,'MarkerSize',8);
plot3(out.fixquadout.Data(end,1), out.fixquadout.Data(end,2), out.fixquadout.Data(end,3),'x','Color',sstgray,'MarkerSize',8);


plot3(out.fixquadout.Data(1,4), out.fixquadout.Data(1,5), out.fixquadout.Data(1,6),'o','Color',sstgray,'MarkerSize',8);
plot3(out.fixquadout.Data(end,4), out.fixquadout.Data(end,5), out.fixquadout.Data(end,6),'x','Color',sstgray,'MarkerSize',8);

    

grid on;
  
xlabel('x [m]');
ylabel('y [m]');
zlabel('-z [m]');

hold off;




 figure(3);
      
     plot(out.fixquadout.Time, out.fixquadout.Data(:,1));
     title('north');
     hold on;
     legend('xfix','xquad');
     plot(out.fixquadout.Time, out.fixquadout.Data(:,4));
     hold off;

      figure(4);
      
     plot(out.fixquadout.Time, out.fixquadout.Data(:,2));
     title('east');
     hold on;
     legend('yfix','y1quad');
     plot(out.fixquadout.Time, out.fixquadout.Data(:,5));
     hold off;

      figure(5);
      
     plot(out.fixquadout.Time, out.fixquadout.Data(:,3));
     title('down');
     hold on;
     legend('zfix','zquad');
     plot(out.fixquadout.Time, out.fixquadout.Data(:,6));
     hold off;

