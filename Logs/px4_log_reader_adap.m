%% Load Data
clc; clear all; close all;
[file_name, path] = uigetfile('.ulg');
file = [path, file_name];

ulog = ulogreader(file);
msg = readTopicMsgs(ulog);
params = readParameters(ulog);
loggedoutput = readLoggedOutput(ulog);
d1 = ulog.StartTime; d2 = ulog.EndTime;
%% Parse the data
data = readTopicMsgs(ulog, 'TopicNames', {'actuator_outputs', 'omni_adap_debug', 'vehicle_visual_odometry' }, 'InstanceID', {2, 0, 0}, 'Time', [d1, d2]);

actuator_outputs = data.TopicMessages{1,1};
omni_debug = data.TopicMessages{2,1};
vehicle_visual_odometry = data.TopicMessages{3,1};
batt_stat = msg.TopicMessages{2,1};

omni_debug.timestamp = omni_debug.timestamp - d1;
vehicle_visual_odometry.timestamp = vehicle_visual_odometry.timestamp - d1;
actuator_outputs.timestamp = actuator_outputs.timestamp - d1;
batt_stat.timestamp = batt_stat.timestamp - d1;
loggedoutput.timestamp = loggedoutput.timestamp - d1;
%% Data Processing
[r, p, y] = quat2angle(omni_debug.attq, 'XYZ');
[r_sp, p_sp, y_sp] = quat2angle(omni_debug.attq_sp, 'XYZ');

a = 1/2 + 1/sqrt(12); b = 1/2 - 1/sqrt(12); c = 1/sqrt(3);
K = 0.019;
K = diag([K,K,K,K,-K,-K,-K,-K]);
J = 0.19;
N = [-a  b  -b  a  a  -b  b  -a;...
     b   a  -a -b -b  -a  a   b;...
     c  -c  -c  c  c  -c  -c  c];
P = 1/sqrt(3)*[1 -1 1 -1 1 -1 1 -1;...
               1 1 -1 -1 1 1 -1 -1;...
               1 1 1 1 -1 -1 -1 -1];

B = [N; J*cross(P, N) + N*K];
B_pinv = pinv(B);

f_rot_act = (B_pinv*[omni_debug.f_cmd, omni_debug.t_cmd]')';

%% Plotting
close all
figure(1)
subplot(3,1,1)
plot(omni_debug.timestamp, r*180/pi, omni_debug.timestamp, r_sp*180/pi)
legend('actual','sp')
ylabel('roll, deg')
% subplot(6,1,2)
% plot(omni_debug.timestamp, omni_debug.omega_cmd(:,1), omni_debug.timestamp, omni_debug.t_cmd(:,1));
% legend('wx','tx')
% ylabel('commands')

subplot(3,1,2)
plot(omni_debug.timestamp, p*180/pi, omni_debug.timestamp, p_sp*180/pi)
legend('actual','sp')
ylabel('pitch, deg')
% subplot(6,1,4)
% plot(omni_debug.timestamp, omni_debug.omega_cmd(:,2), omni_debug.timestamp, omni_debug.t_cmd(:,2));
% legend('wy','ty')
% ylabel('commands')

subplot(3,1,3)
plot(omni_debug.timestamp, y*180/pi, omni_debug.timestamp, y_sp*180/pi)
legend('actual','sp')
ylabel('yaw, deg')
% subplot(6,1,6)
% plot(omni_debug.timestamp, omni_debug.omega_cmd(:,3), omni_debug.timestamp, omni_debug.t_cmd(:,3));
% legend('wz','tz')
% ylabel('commands')
% subtitle('Attitude')

figure(2);
subplot(3,1,1)
plot(omni_debug.timestamp, omni_debug.pos(:,1), vehicle_visual_odometry.timestamp, vehicle_visual_odometry.y, omni_debug.timestamp, omni_debug.pos_sp(:,1))
ylabel('x, m')
subplot(3,1,2)
plot(omni_debug.timestamp, omni_debug.pos(:,2), vehicle_visual_odometry.timestamp, vehicle_visual_odometry.x, omni_debug.timestamp, omni_debug.pos_sp(:,2))
ylabel('y,m')
subplot(3,1,3)
plot(omni_debug.timestamp, omni_debug.pos(:,3), vehicle_visual_odometry.timestamp, -vehicle_visual_odometry.z, omni_debug.timestamp, omni_debug.pos_sp(:,3))
legend('debug','odom')
ylabel('z, m')
subtitle('Position')

figure(3);
for i=1:4
    subplot(4,2,2*i-1)
    plot(actuator_outputs.timestamp, actuator_outputs.output(:,i)); title(['Motor ', num2str(i)]); ylim([900, 2100]); yline(1500, 'k--')
    subplot(4,2,2*i)
    plot(actuator_outputs.timestamp, actuator_outputs.output(:,i+4)); title(['Motor ', num2str(i+4)]); ylim([900, 2100]); yline(1500, 'k--')
end


ti = seconds(omni_debug.timestamp(1));
tf = seconds(omni_debug.timestamp(end));
Xticks = seconds([0:0.25:tf]);

figure(4)
subplot(5,1,1)
plot(omni_debug.timestamp, omni_debug.f_cmd), legend('f_x', 'f_y', 'f_z'), set(gca, 'XTicklabel',[], 'XTick', Xticks), xlim(seconds([ti,tf])); ylabel('Force [N]')
subplot(5,1,2)
plot(omni_debug.timestamp, omni_debug.t_cmd), legend('\tau_x', '\tau_y', '\tau_z'), set(gca, 'XTicklabel',[], 'XTick', Xticks), xlim(seconds([ti,tf])); ylabel('Torque [Nm]')
subplot(5,1,3:4)
plot(omni_debug.timestamp, omni_debug.f_rot_cmd(:,1:4), omni_debug.timestamp, omni_debug.f_rot_filt(:,1:4)), legend('f_r_o_t_, _1', 'f_r_o_t_, _2', 'f_r_o_t_, _3', 'f_r_o_t_, _4'), set(gca, 'XTick', Xticks), xlim(seconds([ti,tf])); ylabel('Force [N]')
hold on; plot(omni_debug.timestamp, omni_debug.f_rot_lim(:,1), 'b:', omni_debug.timestamp, -omni_debug.f_rot_lim(:,1), 'b:');
plot(omni_debug.timestamp, omni_debug.f_rot_lim(:,2), 'r:', omni_debug.timestamp, -omni_debug.f_rot_lim(:,2), 'r:'); ylim([-max(omni_debug.f_rot_lim(:,2)), max(omni_debug.f_rot_lim(:,2))])
subplot(5,1,5)
plot(omni_debug.timestamp, omni_debug.phi(:,1));

figure(5)
subplot(5,1,1)
plot(omni_debug.timestamp, omni_debug.f_cmd), legend('f_x', 'f_y', 'f_z'), set(gca, 'XTicklabel',[], 'XTick', Xticks), xlim(seconds([ti,tf])); ylabel('Force [N]')
subplot(5,1,2)
plot(omni_debug.timestamp, omni_debug.t_cmd), legend('\tau_x', '\tau_y', '\tau_z'), set(gca, 'XTicklabel',[], 'XTick', Xticks), xlim(seconds([ti,tf])); ylabel('Torque [Nm]')
subplot(5,1,3:4)
plot(omni_debug.timestamp, omni_debug.f_rot_cmd(:,5:8), omni_debug.timestamp, omni_debug.f_rot_filt(:,5:8)), legend('f_r_o_t_, _5', 'f_r_o_t_, _6', 'f_r_o_t_, _7', 'f_r_o_t_, _8'), set(gca, 'XTick', Xticks), xlim(seconds([ti,tf])); ylabel('Force [N]')
hold on; plot(omni_debug.timestamp, omni_debug.f_rot_lim(:,1), 'b:', omni_debug.timestamp, -omni_debug.f_rot_lim(:,1), 'b:');
plot(omni_debug.timestamp, omni_debug.f_rot_lim(:,2), 'r:', omni_debug.timestamp, -omni_debug.f_rot_lim(:,2), 'r:'); ylim([-max(omni_debug.f_rot_lim(:,2)), max(omni_debug.f_rot_lim(:,2))])
subplot(5,1,5)
plot(omni_debug.timestamp, omni_debug.phi(:,2));

figure(6)
ha = tight_subplot(2,1, .01, .05, .05);
axes(ha(1))
plot(omni_debug.timestamp, omni_debug.omega*180/pi); set(gca, 'XTicklabel',[], 'XTick', Xticks), ylabel 'Angular Velocity [°/s]', legend('roll speed', 'pitch speed', 'yaw speed')
axes(ha(2))
plot(omni_debug.timestamp, omni_debug.t_cmd); set(gca, 'XTick', Xticks), ylabel 'Body Torque [N-m]', legend('\tau_x', '\tau_y', '\tau_z')

figure(7);
ha = tight_subplot(3,1, .01, .05, .05);
axes(ha(1))
plot(omni_debug.timestamp, r*180/pi); set(gca, 'XTicklabel',[], 'XTick', Xticks), title 'Roll', ylabel 'Angle [°]'
axes(ha(2))
plot(omni_debug.timestamp, omni_debug.omega(:,1)*180/pi, omni_debug.timestamp, omni_debug.omega_cmd(:,1)*180/pi); legend('\omega', '\omega_s_p'), set(gca, 'XTicklabel',[], 'XTick', Xticks), ylabel 'Angular rate [°/s]'
axes(ha(3))
plot(omni_debug.timestamp, omni_debug.t_cmd(:,1)), set(gca, 'XTick', Xticks), ylabel 'Torque [N-m]'

figure(8);
ha = tight_subplot(3,1, .01, .05, .05);
axes(ha(1))
plot(omni_debug.timestamp, p*180/pi); set(gca, 'XTicklabel',[], 'XTick', Xticks), title 'Pitch', ylabel 'Angle [°]'
axes(ha(2))
plot(omni_debug.timestamp, omni_debug.omega(:,2)*180/pi, omni_debug.timestamp, omni_debug.omega_cmd(:,2)*180/pi); legend('\omega', '\omega_s_p'), set(gca, 'XTicklabel',[], 'XTick', Xticks), ylabel 'Angular rate [°/s]'
axes(ha(3))
plot(omni_debug.timestamp, omni_debug.t_cmd(:,2)), set(gca, 'XTick', Xticks), ylabel 'Torque [N-m]'
%% Plot Testing


figure(9);
ha = tight_subplot(3,1, .01, .05, .05);
axes(ha(1))
plot(omni_debug.timestamp, y*180/pi); set(gca, 'XTicklabel',[], 'XTick', Xticks), title 'Yaw', ylabel 'Angle [°]'
axes(ha(2))
plot(omni_debug.timestamp, omni_debug.omega(:,3)*180/pi, omni_debug.timestamp, omni_debug.omega_cmd(:,3)*180/pi); legend('\omega', '\omega_s_p'), set(gca, 'XTicklabel',[], 'XTick', Xticks), ylabel 'Angular rate [°/s]'
axes(ha(3))
plot(omni_debug.timestamp, omni_debug.t_cmd(:,3)), set(gca, 'XTick', Xticks), ylabel 'Torque [N-m]'

figure(10)
tiledlayout(3,1)

nexttile
plot(omni_debug.timestamp, omni_debug.x(:,1:3))
hold on
plot(omni_debug.timestamp,omni_debug.xr(:,1:3),'--')
plot(omni_debug.timestamp,omni_debug.pos,'-.')
legend('x','y','z','xr','yr','zr','x','y','z')
nexttile
plot(omni_debug.timestamp, omni_debug.x(:,4:6))
hold on
plot(omni_debug.timestamp,omni_debug.xr(:,4:6),'--')
legend('vx','vy','vz','vxr','vyr','vzr')
nexttile
plot(omni_debug.timestamp, omni_debug.x(:,7:9))
hold on
plot(omni_debug.timestamp,omni_debug.xr(:,7:9),'--')
legend('exI','eyI','ezI','exIr','eyIr','ezIr')

figure(11)
plot(omni_debug.timestamp,vecnorm(omni_debug.theta_pid_one,2,2))
hold on
plot(omni_debug.timestamp,vecnorm(omni_debug.theta_pid_two,2,2))
plot(omni_debug.timestamp,vecnorm(omni_debug.theta_pid_three,2,2))
legend('col1','col2','col3')
title('aMRAC Theta Norms')

figure(12)
tiledlayout(2,1)
nexttile
plot(omni_debug.timestamp,omni_debug.f_rot_cmd(:,1:4))
hold on
plot(omni_debug.timestamp,omni_debug.f_rot_filt(:,1:4))
hold off
legend('f1','f2','f3','f4','f1est','f2est','f3est','f4est')

nexttile
plot(omni_debug.timestamp,omni_debug.f_rot_cmd(:,5:end))
hold on
plot(omni_debug.timestamp,omni_debug.f_rot_filt(:,5:end))
hold off
legend('f5','f6','f7','f8','f5est','f6est','f7est','f8est')

figure(13)
tiledlayout(3,1)
nexttile
plot(omni_debug.timestamp, omni_debug.tbl)
legend('t1','t2','t3')
nexttile
plot(omni_debug.timestamp, omni_debug.tad)
legend('t1','t2','t3')
nexttile
plot(omni_debug.timestamp, omni_debug.t_cmd)
legend('t1','t2','t3')

figure(26)
plot(omni_debug.timestamp,omni_debug.logv)
legend('t1','t2','t3')

%%
% m = 1.172; %kg
% Tpos = 0.325; %s, position control time constant
% Tpos_i = 1.33; %s, position control integral time constant
% zeta_pos = 1.0; %position control damping ratio
% 
% %baseline model: position controller
% Ap = [zeros(3), eye(3); zeros(3,6)]; %This should only be the part of the A matrix and B matrix for what is tracked???
% Bp = 1/m*[zeros(3); eye(3)]; %1/m? I think
% 
% %augmented model
% A = [Ap zeros(6,3); eye(3,6) zeros(3)];
% B = [Bp; zeros(3)];
% Bref = [zeros(6,3); - eye(3)]; %works for the attitude controller as well
% 
% Kp = 1/Tpos^2+(2*zeta_pos)/(Tpos*Tpos_i);
% Kd = 2*zeta_pos/Tpos+1/Tpos_i;
% Ki = 1/(Tpos^2*Tpos_i);
% Kpid = m*[Kp*eye(3); Kd*eye(3); Ki*eye(3)];
% 
% ArefPID = A-B*Kpid';
% 
% %Adaptive Parameters
% GammaPID = 200*diag([1,1,1]); %gain
% 
% Qpid = diag([1 1 1 1 1 1 1 1 1]); %Lyap parameters
% Ppid = lyap(ArefPID',Qpid);
% 
% for i = 1:length(omni_debug.x(:,1))
% f_bl(1:3,i) = -Kpid'*(omni_debug.x(i,:)'-[omni_debug.pos_sp(i,:)';zeros(6,1)]);
% e(1:9,i) = omni_debug.x(i,:)'-[omni_debug.pos_sp(i,:)';zeros(6,1)];
% end


% figure(12)
% plot(1:length(omni_debug.x(:,1)),f_bl)

% figure(13)
% tiledlayout(3,1)
% nexttile
% plot(omni_debug.timestamp,e(1:3,:))
% hold on 
% %plot(omni_debug.timestamp, f_bl(3,:))
% ylim([-0.5 0.5])
% legend('ex','ey','ez')
% 
% nexttile
% plot(omni_debug.timestamp,e(4:6,:))
% hold on 
% %plot(omni_debug.timestamp, f_bl(3,:))
% ylim([-0.5 2])
% legend('evx','evy','evz')
% 
% nexttile
% plot(omni_debug.timestamp,e(7:9,:))
% hold on 
% %plot(omni_debug.timestamp, f_bl(3,:))
% ylim([-0.5 0.5])
% legend('evxI','evyI','evzI')

%%
figure(14)
tiledlayout(3,1)
nexttile
plot(omni_debug.timestamp,omni_debug.fbl)
legend('fblx','fbly','fblz')
nexttile
plot(omni_debug.timestamp,omni_debug.x(:,1:3)-omni_debug.xr(:,1:3))
legend('ea','eay','eaz')
nexttile
plot(omni_debug.timestamp,omni_debug.fad)
legend('fadx','fady','fadz')
%ylim([-100 200])

figure(15)
tiledlayout(3,1)
nexttile
plot(omni_debug.timestamp,omni_debug.xdbar(:,1:3))
legend('xdx','xdy','xdz')
nexttile
plot(omni_debug.timestamp,omni_debug.xdbar(:,4:6))
legend('xdvx','xdvy','xdvz')
nexttile
plot(omni_debug.timestamp,omni_debug.xdbar(:,7:9))
legend('xdex','xdey','xdez')
%ylim([-100 200])

figure(16)
tiledlayout(3,1)
nexttile
plot(omni_debug.timestamp,omni_debug.xd(:,1:3))
legend('xdx','xdy','xdz')
nexttile
plot(omni_debug.timestamp,omni_debug.xd(:,4:6))
legend('xdvx','xdvy','xdvz')
nexttile
plot(omni_debug.timestamp,omni_debug.xd(:,7:9))
legend('xdex','xdey','xdez')

figure(17)
tiledlayout(2,1)
nexttile
plot(omni_debug.timestamp,omni_debug.pos_sp)
hold on
plot(omni_debug.timestamp,omni_debug.pos)
hold off
legend('x_sp','y_sp','z_sp','x','y','z')
nexttile
plot(omni_debug.timestamp,omni_debug.vel_sp)
hold on
plot(omni_debug.timestamp,omni_debug.vel)
hold off
legend('vx_sp','vy_sp','vz_sp','vx','vy','vz')


figure(18)
tiledlayout(3,1)
nexttile
plot(omni_debug.timestamp, r*180/pi) %, omni_debug.timestamp, r_sp*180/pi)
hold on
plot(omni_debug.timestamp, p*180/pi) % omni_debug.timestamp, p_sp*180/pi)
plot(omni_debug.timestamp, y*180/pi) % omni_debug.timestamp, p_sp*180/pi)
legend('roll','pitch','yaw')
hold off
nexttile
hold on
plot(omni_debug.timestamp, omni_debug.omega(:,1));
plot(omni_debug.timestamp, omni_debug.omega(:,2));
plot(omni_debug.timestamp, omni_debug.omega(:,3));
legend('wx','wy','wz')
ylim([-5 5])
hold off
nexttile
plot(omni_debug.timestamp, omni_debug.t_cmd(:,1));
hold on
plot(omni_debug.timestamp, omni_debug.t_cmd(:,2));
plot(omni_debug.timestamp, omni_debug.t_cmd(:,3));
legend('tx','ty', 'tz')
hold off

figure(19)
plot(omni_debug.timestamp,vecnorm(omni_debug.theta_ab_one,2,2))
hold on
plot(omni_debug.timestamp,vecnorm(omni_debug.theta_ab_two,2,2))
plot(omni_debug.timestamp,vecnorm(omni_debug.theta_ab_three,2,2))
hold off
legend('col1','col2','col3')
title('aBack Theta Norms')
ylim([0 2])

figure(19)
tiledlayout(3,1)
nexttile
plot(omni_debug.timestamp,vecnorm(omni_debug.theta_att_one,2,2))
hold on
plot(omni_debug.timestamp,vecnorm(omni_debug.theta_att_two,2,2))
plot(omni_debug.timestamp,vecnorm(omni_debug.theta_att_three,2,2))
hold off
legend('col1','col2','col3')
title('Theta Norms')
ylim([0 2])

nexttile
plot(omni_debug.timestamp,vecnorm(omni_debug.lambdahat_one,2,2))
hold on
plot(omni_debug.timestamp,vecnorm(omni_debug.lambdahat_two,2,2))
plot(omni_debug.timestamp,vecnorm(omni_debug.lambdahat_three,2,2))
hold off
legend('col1','col2','col3')
title('Lambda Norms')

nexttile
plot(omni_debug.timestamp,vecnorm(omni_debug.tdhat,2,2))
title('tdhat Norms')

r2d = 180/pi;

figure(20)
tiledlayout(2,1)
nexttile
hold on
plot(omni_debug.timestamp, omni_debug.omega_sp(:,1)*r2d);
plot(omni_debug.timestamp, omni_debug.omega_sp(:,2)*r2d);
plot(omni_debug.timestamp, omni_debug.omega_sp(:,3)*r2d);

plot(omni_debug.timestamp, omni_debug.omega(:,1)*r2d);
plot(omni_debug.timestamp, omni_debug.omega(:,2)*r2d);
plot(omni_debug.timestamp, omni_debug.omega(:,3)*r2d);
legend('wx_sp','wy_sp','wz_sp','wx','wy','wz')
ylim([-100 100])
hold off

nexttile
hold on
plot(omni_debug.timestamp, omni_debug.omega_dot_sp(:,1));
plot(omni_debug.timestamp, omni_debug.omega_dot_sp(:,2));
plot(omni_debug.timestamp, omni_debug.omega_dot_sp(:,3));
legend('wdotx_sp','wdoty_sp','wdotz_sp')
hold off


figure(21)
plot(omni_debug.timestamp, omni_debug.u)
hold on
plot(omni_debug.timestamp, vecnorm(omni_debug.u'))
legend('u1','u2','u3','norm')

figure(22)
plot(omni_debug.timestamp,omni_debug.attq_sp)
hold on
plot(omni_debug.timestamp,omni_debug.attq)
hold off
legend('q0sp','q1sp','q2sp','q3sp','q0','q1','q2','q3')

figure(24)
plot3(omni_debug.pos_sp(:,1),omni_debug.pos_sp(:,2),omni_debug.pos_sp(:,3))
hold on
plot3(omni_debug.pos(:,1),omni_debug.pos(:,2),omni_debug.pos(:,3))
hold off
xlabel('x')
ylabel('y')
zlabel('z')
%%
posTime = sqrt(sum(omni_debug.pos_error.^2,2));
posErr = mean(posTime) %m
posStr = sprintf('Avg Pos Err: %0.3f m', posErr);

for i = 1:length(omni_debug.timestamp)
    qTime(i) = acos(2*dot(omni_debug.attq(i,:),omni_debug.attq_sp(i,:)/norm(omni_debug.attq_sp(i,:))).^2-1); %radians, 
end
qErr = mean(qTime)
qStr = sprintf('Avg Att Err: %0.3f deg', qErr*180/pi);

dim1 = [.2 .7 .15 .15];
dim2 = [.6 .7 .15 .15];

figure(23)
tiledlayout(1,2)
nexttile
plot(omni_debug.timestamp, posTime)
annotation('textbox',dim1,'String',posStr,'FitBoxToText','on')
xlabel('time (s)')
ylabel('error (m)')
title('Position Error')

nexttile
plot(omni_debug.timestamp, qTime*180/pi)
annotation('textbox',dim2,'String',qStr,'FitBoxToText','on')
xlabel('time (s)')
ylabel('error (deg)')
title('Attitude Error')


%%
figure(25)
    tiledlayout(3,2)
    nexttile(1)
    plot(omni_debug.timestamp,omni_debug.pos_sp)
    legend('x','y','z')
    ylabel('position, m')
    grid on
    
    nexttile(3)
    plot(omni_debug.timestamp,omni_debug.vel_sp)
    legend('vx','vy','vz')
    ylabel('velocity, m/s')
    xlabel('time, s')
    grid on
    
    nexttile(2)
    plot(omni_debug.timestamp,omni_debug.attq_sp)
    legend('q0','q1','q2','q3')
    ylabel('quaternion')
    grid on
    
    nexttile(4)
    plot(omni_debug.timestamp,omni_debug.omega_sp)
    legend('wx','wy','wz')
    ylabel('angular velocity, rad/s')
    grid on
    
    nexttile(6)
    plot(omni_debug.timestamp,omni_debug.omega_dot_sp)
    legend('wdot_x','wdot_y','wdot_z')
    ylabel('angular accel, rad/s^2')
    xlabel('time, s')
    grid on
    sgtitle('Setpoints')
    
    
 figure(27)
 plot(omni_debug.timestamp, omni_debug.q_error)
 legend('q0','q1','q2','q3')

 power_consumption = sum((hours(batt_stat.timestamp(1:end)) - [0; hours(batt_stat.timestamp(1:end-1))]).*batt_stat.voltage_filtered_v.*batt_stat.current_filtered_a);
 dim3 = [.25 .25 .15 .15];
 powStr = sprintf('Power Consumption: %0.3f Wh', power_consumption);

 figure(28)
 plot(batt_stat.timestamp, batt_stat.voltage_filtered_v.*batt_stat.current_filtered_a)
 annotation('textbox',dim3,'String',powStr,'FitBoxToText','on')
 xlabel('Time [s]'), ylabel('Power [W]')