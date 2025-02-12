function [posMat,t] = smoothTraj(type,startPos,n,segTime,dt,seed)

switch type
    case 'random'
        if seed ~= []
            rng(seed)
        end
        vals = round([-5.5/2 + 5.5*rand(n,1), -.5 + 2.5*rand(n,1), .75+2*rand(n,1)],2);
    case 'preset'
%         x = [startPos(1) .75, 2.25, 1.25, .25, -1.5,-1.25 startPos(1)];
%         y = [startPos(2) -.75, 0, .75, -.65, -.5, .25 startPos(2)];
%         z = [startPos(3) 1.75 1.35, .9 1.15 1.75 1 startPos(3)];
        x = [startPos(1) .75, 2.5, 1.25, -0.75, 0.25, 1.35, 0, -0.85, startPos(1)];
        y = [startPos(2) -.75, 0, 1, 0, -0.9, 0, 0.85, 0.35, startPos(2)];
        z = [startPos(3) 1.5 .8 1.5 0.65 1.2 1.85, 1.55, 1.15 startPos(3)];
        vals = [x',y',z'];
end

n = length(x);

toa = 0:segTime:(segTime)*(n+1);

[Pos,Vel,Accel,~,~,~,~,t] = minsnappolytraj(vals', [0:segTime:(n-1)*segTime], 1/dt*(n-1)*segTime+1);

% traj = waypointTrajectory(vals,'TimeOfArrival',toa,'SampleRate',1/dt);
% 
% Pos = zeros(toa(end)*traj.SampleRate+1,3);
% Vel = zeros(toa(end)*traj.SampleRate+1,3);
% Accel = zeros(toa(end)*traj.SampleRate+1,3);
% 
% count = 1;
% while ~isDone(traj)
%     [Pos(count+1,:),~,Vel(count+1,:),Accel(count+1,:),~] = traj();
%     count = count+1;
% end
% 
% Pos(1,:) = startPos';
% Pos(end,:) = startPos';
% 
% figure(4)
% plot3(vals(:,1),vals(:,2),vals(:,3),'*-')
% hold on
% plot3(Pos(:,1),Pos(:,2),Pos(:,3))
% xlim([-3.25 3.25])
% ylim([-1.25 2.25])
% zlim([.5 3])
% xlabel('x')
% ylabel('y')
% zlabel('z')

posMat = [Pos',Vel',Accel'];
% t =  0:dt:toa(end);
end