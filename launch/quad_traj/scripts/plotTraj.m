%posTraj [array]: Nx4 [time x y z]
%attTraj [array]: Nx5, [time, q0 qx qy qz]

function plotTraj(t,posTraj,attTraj,deltat)
x = posTraj(:,1);
y = posTraj(:,2);
z = posTraj(:,3);

q0 = attTraj(:,1);
qvec = attTraj(:,2:4);

xvec = [1;0;0]; %aligned with body frame
yvec = [0;1;0];
zvec = [0;0;1];

h = figure(3);
plot3(x, y, z, 'LineWidth', 1.5) %plot the full trajectory
%view(2) %xy
%view(90,0) %zy
%view(0,90)
view
grid on
hold on
axis equal tight manual
% zlim([0 2.5])
% ylim([-1 1])
% xlim([-1 1])
xlabel('X, meters')
ylabel('Y, meters')
zlabel('Z, meters')
point = plot3(x(1), y(1), z(1), '.', 'MarkerSize', 20); %plot the location on the trajectory

R = (q0(1)^2-qvec(1,:)*qvec(1,:)')*eye(3)+2*(qvec(1,:)'*qvec(1,:)-q0(1)*crossMat(qvec(1,:))); %rotation matrix frame inertial to body frame

%rotate from body to inertial frame
xbod = R\xvec; %red
ybod = R\yvec; %blue
zbod = R\zvec; %green

xquiv = quiver3(x(1), y(1), z(1), xbod(1), xbod(2), xbod(3), 1, 'r', 'LineWidth', 1.5); %positions/pointing occurs in the inertial frame
yquiv = quiver3(x(1), y(1), z(1), ybod(1), ybod(2), ybod(3),1, 'b', 'LineWidth', 1.5);
zquiv = quiver3(x(1), y(1), z(1), zbod(1), zbod(2), zbod(3), 1, 'g', 'LineWidth', 1.5);
%axisquiver1 = quiver3(x(1), y(1), z(1),1,1,1,'k','Linewidth',1.5);
%axisquiver1 = quiver3(x(1), y(1), z(1),-1,-1,-1,'k','Linewidth',1.5);

frame = getframe(h);
im = frame2im(frame);
[imind,cm] = rgb2ind(im,256);
for i = 2:length(t)-1
    point.XData = x(i);
    point.YData = y(i);
    point.ZData = z(i);
    
    R = (q0(i)^2-qvec(i,:)*qvec(i,:)')*eye(3)+2*(qvec(i,:)'*qvec(i,:)-q0(i)*crossMat(qvec(i,:)));
    xbod = R\xvec; %red
    ybod = R\yvec; %blue
    zbod = R\zvec; %green
    
    xquiv.XData = x(i);
    xquiv.YData = y(i);
    xquiv.ZData = z(i);
    xquiv.UData = xbod(1);
    xquiv.VData = xbod(2);
    xquiv.WData = xbod(3);
    
    yquiv.XData = x(i);
    yquiv.YData = y(i);
    yquiv.ZData = z(i);
    yquiv.UData = ybod(1);
    yquiv.VData = ybod(2);
    yquiv.WData = ybod(3);
    
    zquiv.XData = x(i);
    zquiv.YData = y(i);
    zquiv.ZData = z(i);
    zquiv.UData = zbod(1);
    zquiv.VData = zbod(2);
    zquiv.WData = zbod(3);
    
    frame(i) = getframe(h);
    im = frame2im(frame(i));
    [imind,cm] = rgb2ind(im,256);
    pause(0.01)
end

for i = 2.5/deltat:2.5/deltat:length(t)-1 %want to freeze frame each quiver
    hold on
    point.XData = x(i);
    point.YData = y(i);
    point.ZData = z(i);
    
    R = (q0(i)^2-qvec(i,:)*qvec(i,:)')*eye(3)+2*(qvec(i,:)'*qvec(i,:)-q0(i)*crossMat(qvec(i,:)));
    xbod = R\xvec; %red
    ybod = R\yvec; %blue
    zbod = R\zvec; %green
    
    a = quiver3(x(i),y(i),z(i),xbod(1),xbod(2),xbod(3),'r');
    b = quiver3(x(i),y(i),z(i),ybod(1),ybod(2),ybod(3),'b');
    c = quiver3(x(i),y(i),z(i),zbod(1),zbod(2),zbod(3),'g');
    
    set(a,'AutoScale','on', 'AutoScaleFactor',0.25)
    set(b,'AutoScale','on', 'AutoScaleFactor',0.25)
    set(c,'AutoScale','on', 'AutoScaleFactor',0.25)
    
end

%  % create the video writer with 1 fps
%   writerObj = VideoWriter('axisRot.avi');
%   %writerObj.FrameRate = .01;
%   % set the seconds per image
% % open the video writer
% open(writerObj);
% % write the frames to the video
% for i=1:length(frame)
%     % convert the image to a frame
%     F = frame(i) ;    
%     writeVideo(writerObj, frame);
% end
% % close the writer object
% close(writerObj);