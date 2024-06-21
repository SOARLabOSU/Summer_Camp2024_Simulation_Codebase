function  quad_animation(x,y,z,roll,pitch,yaw,xd,video_flag)
% This Animation code is for QuadCopter. Written by Jitendra Singh 

%% Define design parameters
D2R = pi/180;
R2D = 180/pi;
b   = 0.6;   % the length of total square cover by whole body of quadcopter in meter
a   = b/3;   % the legth of small square base of quadcopter(b/4)
H   = 0.06;  % hight of drone in Z direction (4cm)
H_m = H+H/2; % hight of motor in z direction (5 cm)
r_p = b/4;   % radius of propeller
%% Conversions
ro = 45*D2R;                   % angle by which rotate the base of quadcopter
Ri = [cos(ro) -sin(ro) 0;
      sin(ro) cos(ro)  0;
       0       0       1];     % rotation matrix to rotate the coordinates of base 
base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base 
           -a/2 -a/2 a/2 a/2;
             0    0   0   0];
base = Ri*base_co;             % rotate base Coordinates by 45 degree 

to = linspace(0, 2*pi);
xp = r_p*cos(to);
yp = r_p*sin(to);
zp = zeros(1,length(to));

 %% Init Animation

 if video_flag
     numberOfFrames=length(x);
    allTheFrames = cell(numberOfFrames,1);
    vidHeight = 412;
    vidWidth = 622;
    allTheFrames(:) = {zeros(vidHeight, vidWidth, 3, 'uint8')};
    % Next get a cell array with all the colormaps.
    allTheColorMaps = cell(numberOfFrames,1);
    allTheColorMaps(:) = {zeros(256, 3)};
    % Now combine these to make the array of structures.
    myMovie = struct('cdata', allTheFrames, 'colormap', allTheColorMaps);
    % Create a VideoWriter object to write the video out to a new, different file.
    % writerObj = VideoWriter('problem_3.avi');
    % open(writerObj);
    % Need to change from the default renderer to zbuffer to get it to work right.
    % openGL doesn't work and Painters is way too slow.
 end
%% Define Figure plot
 fig1 = figure('pos', [0 50 800 600]);
 set(gcf, 'renderer', 'zbuffer');
 hg   = gca;
 view(-40,20);
 grid on;
 axis equal;
 xlim([min(x)-0.6 max(x)+0.6]); 
 ylim([min(y)-0.6 max(y)+0.6]); 
 zlim([min(z)-0.6 max(z)+0.6]);


 xlabel('X[m]');
 ylabel('Y[m]');
 zlabel('Z[m]');
 hold(gca, 'on');


%% Design Different parts
% design the base square
 drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
 drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
 alpha(drone(1:2),0.7);
% design 2 parpendiculer legs of quadcopter 
 [xcylinder ycylinder zcylinder] = cylinder([H/2 H/2]);
 drone(3) =  surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
 drone(4) =  surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b') ; 
 alpha(drone(3:4),0.6);
% design 4 cylindrical motors 
 drone(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
 drone(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
 drone(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
 drone(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');
 alpha(drone(5:8),0.7);
% design 4 propellers
 drone(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
 drone(10) = patch(xp-b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
 drone(11) = patch(xp,yp+b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
 drone(12) = patch(xp,yp-b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
 alpha(drone(9:12),0.3);

%% create a group object and parent surface
  combinedobject = hgtransform('parent',hg );
  set(drone,'parent',combinedobject)
%  drawnow
 
 for i = 1:length(x)
  
     ba = plot3(x(1:i),y(1:i),z(1:i), 'b--','LineWidth',2);
     % xlim([-0.6 2.2]); ylim([-0.6 1]); zlim([-0.6 1]);
     plot3(0, 0, 0, '*r', 'MarkerSize', 10);
     plot3(xd(1), xd(2), xd(3), '*g', 'MarkerSize', 12);
     translation = makehgtform('translate',...
                               [x(i) y(i) z(i)]);
     %set(combinedobject, 'matrix',translation);
     rotation3 = makehgtform('zrotate',yaw(i));
     rotation2 = makehgtform('yrotate',(pitch(i)));
     rotation1 = makehgtform('xrotate',(roll(i)));
     
     
     %scaling = makehgtform('scale',1-i/20);
     set(combinedobject,'matrix',...
          translation*rotation3*rotation2*rotation1);
      
      %movieVector(i) =  getframe(fig1);
        %delete(b);
        % legend('Trajectory','Start','Target')
     drawnow
     % pause;
   pause(0.1);
    
     if video_flag
        thisFrame = getframe(fig1);
	    % Write this frame out to a new video file.
        %  	writeVideo(writerObj, thisFrame);
        if i <= numberOfFrames 
	        myMovie(i) = thisFrame;
        end
     end
 end

 %% writing as a video
if video_flag
    writerObj = VideoWriter('mymov.mp4', 'MPEG-4');
    writerObj.Quality = 95;
    open(writerObj);
    % Write out all the frames.
    numberOfFrames = length(myMovie);
    for frameNumber = 1 : numberOfFrames 
       writeVideo(writerObj, myMovie(frameNumber));
    end
    close(writerObj);
end

 
