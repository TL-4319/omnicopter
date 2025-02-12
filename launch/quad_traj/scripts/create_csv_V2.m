clear all; close all; clc;
%% Settings
%time
tend = (360*2)/15; %seconds, determines speed of traj
dt = 1/50; %match 50 Hz publishing rate
t = 0:dt:tend;
transTime = 2; %hover two seconds before 

% save settings

filename = 'smooth_point_V3'; %'inc25_ellipse_roll_50Hz';


%position settings
posStyle = 'preset'; % x,y,z,hover,ellipse,preset,random
height = 1; %starting height of trajectories
xbound = [-2 3];
ybound = [-1.5 1.5];
zbound = [0 2];
startPos = [0,0,height]; %where you want your trajectory to start

%for ellipse
inc = 30*pi/180; %inclination of circle
r = 1; %m %radius of circle
a = 2/2; %semimajor axis, (x direction)
b = 2/2; %semiminor axis,(y direction)
e = 0; %sqrt(1-b^2/a^2);

%for smoothTraj
npoints = 7;
segTime = 8;
seed = [];

%attitude settings
attStyle = 'aim'; % roll, pitch, yaw, +x, ang aim
deg = 360; %angle to roll,pitch, or yaw between or to point at
aimVec = [1.25,0,1.25]; %aim point for the +x face
ax = [1,1,1]/norm([1,1,1]);
%% Position Trajectory
limPos = [xbound;ybound;zbound];

switch posStyle 
    case {'x','y','z','hover'}
        [posMat t] = createLine(posStyle,startPos,limPos,t,dt); %hover,x,y,z,'+xy','-xy','+xz,'-xz'
    case 'ellipse'
        [posMat t]= createEllipse(inc,startPos,dt,t,a,e);
    case {'preset','random'}
        [posMat t] = smoothTraj(posStyle,startPos,npoints,segTime,dt,seed);
%posMat = createSquare();
end

%% Attitude Trajectory
switch attStyle
    case {'roll','pitch','yaw','axis'}
        [attMat t] = rotations(attStyle,deg,t,dt,ax); %roll,pitch,yaw,axis
        [posMat t] = createLine(posStyle,startPos,limPos,t,dt); %hover
    case 'aim'
        attMat = aim(aimVec,posMat,segTime,dt,t);
    case {'+x','ang'}
        attMat = constantPoint(attStyle,t,deg); %+x,+y,+z, tell which way for x face to point
end

%% Plot
figure(1) 
tiledlayout(3,2)
nexttile(1)
plot(t,posMat(:,1:3))
legend('x','y','z')
ylabel('position, m')
nexttile(3)
plot(t,posMat(:,4:6))
legend('vx','vy','vz')
ylabel('velcoity, m/s')
nexttile(5)
plot(t,posMat(:,7:9))
legend('ax','ay','az')
ylabel('acceleration, m/s^2')
xlabel('time,s')

nexttile(2)
plot(t,attMat(:,1:4))
legend('q0','qx','qy','qz')
ylabel('attitude')
nexttile(4)
plot(t,attMat(:,5:7))
legend('wx','wy','wz')
ylabel('angular velocity, rad/s')
xlabel('time,s')
nexttile(6)
plot(t, (attMat(1:end,5:7) - [0 0 0; attMat(1:end-1,5:7)])/dt)
ylabel('angular acceleration [rad/s^2]')
xlabel('time [s]')
% plot(t,quat2eul(attMat(:,1:4),'XYZ')*180/pi)

figure(2)
plot3(posMat(:,1),posMat(:,2),posMat(:,3))
xlabel('x')
ylabel('y')
zlabel('z')
hold on
plot3(posMat(1,1),posMat(1,2),posMat(1,3),'g*','Linewidth',3)
plot3(posMat(100,1),posMat(100,2),posMat(100,3),'r*','Linewidth',3)
plot3(aimVec(1),aimVec(2),aimVec(3),'b*','Linewidth',4)
hold off
grid on

axis equal
xlim(xbound), ylim(ybound), zlim(zbound)


%%
% switch control
%     case 'PID'
%         M = [posMat(:,[1:3,7:9]) attMat];
%         csvwrite([filename '_accel'],M); %PID needs accel setpoint
%     case 'aMRAC'
%         M = [posMat(:,[1:6]) attMat];
%         csvwrite([filename '_vel'],M); %aMRAC needs vel setpoint
% end

M = [posMat attMat];
csvwrite(filename,M);