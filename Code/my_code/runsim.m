close all
clear all

addpath('trajectories')

% trajectory generator
trajhandle = @step;
% trajhandle = @circle;
% trajhandle = @diamond;
% trajhandle = @hover_trajectory;

% controller
% controlhandle = @controller;
controlhandle = @geometric_controller;

% real-time 
real_time = false;
% Visualization
vis_init  = false;

% max time
time_tol = 15;

% parameters for simulation
params = fpvdrone();
nprop = 40;
ifov   = 90;          % Camera field of view  
propangs = linspace(0,2*pi,nprop);

% **************************** FIGURES *****************************
fprintf('Initializing figures...\n')
h_fig = figure;
h_3d = gca;
axis equal
grid on
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')

set(gcf,'Renderer','OpenGL')

% *********************** INITIAL CONDITIONS ***********************
fprintf('Setting initial conditions...\n')
max_iter  = 5000;      % max iteration
starttime = 0;         % start of simulation in seconds
tstep     = 0.002;      % this determines the time step at which the solution is given
cstep     = 0.01;      % image capture time interval
nstep     = cstep/tstep;
time      = starttime; % current time
err = []; % runtime errors

% Get start and stop position
des_start = trajhandle(0);
des_stop  = trajhandle(inf);
stop  = des_stop.pos;
x0    = init_state( des_start.pos, 0 );
xtraj = zeros(max_iter*nstep, length(x0));
ttraj = zeros(max_iter*nstep, 1);

qd.state_list=zeros(max_iter,16);
qd.state_des_list=zeros(max_iter,13);
qd.time_list=zeros(max_iter,1);
qd.k=0;

x         = x0;        % state

pos_tol   = 0.01;
vel_tol   = 0.01;


% ************************* RUN SIMULATION *************************
fprintf('Simulation Running....')
% Main loop
for iter = 1:max_iter
    iter;
    timeint = time:tstep:time+cstep;

    tic;
    % Run simulation
    qd.pos = x(1:3);
    qd.vel = x(4:6);
    qd.quat = x(7:10);
    qd.Rot = QuatToRot(qd.quat);

    [phi, theta, yaw] = RotToRPY_ZYX(qd.Rot);

    qd.euler = [phi; theta; yaw];
    qd.omega = x(11:13);
    desired_state = trajhandle(time);
    % The desired_state is set in the trajectory generator
    qd.pos_des      = desired_state.pos;
    qd.vel_des      = desired_state.vel;
    qd.acc_des      = desired_state.acc;
    qd.yaw_des      = desired_state.yaw;
    qd.yawdot_des   = desired_state.yawdot;
    qd.state_list(iter,1:13)=x;
    qd.state_list(iter,14:16)=qd.euler;
    qd.state_des_list(iter,1:3)=qd.pos_des;
    qd.state_des_list(iter,4:6)=qd.vel_des;
    qd.state_des_list(iter,7:9)=qd.acc_des;
    
    % get control outputs
    [F, M, trpy, drpy] = controlhandle(qd, time, params);
    qd.pitch_des = trpy(3);
    qd.roll_des = trpy(2);
    qd.state_des_list(iter,10)=qd.roll_des;
    qd.state_des_list(iter,11)=qd.pitch_des;
    qd.state_des_list(iter,12)=qd.yaw_des;
    qd.state_des_list(iter,13)=qd.yawdot_des;
    qd.time_list(iter)=time;
    if 0.4<qd.pos(2)<0.6
        F_disturbance = [0;0;0];
    else
        F_disturbance = [0;0;0];
    end
    [tsave, xsave] = ode45(@(t,s) quadEOM_readonly(t, s, F, M, params, F_disturbance), timeint, x);
    x    = xsave(end, :)';
%     x(1:3)=x(1:3)+randn(3,1)*0.001;
    
    
    % Save to traj
    xtraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
    ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);
    
    time = time + cstep; % Update simulation time
    qd.k = qd.k + 1;
    t = toc
    % Check to make sure ode45 is not timing out
    if(t> cstep*50)
        err = 'Ode45 Unstable';
        break;
    end

    % Check termination criteria
    if terminate_check(x, time, stop, pos_tol, vel_tol, time_tol)
        break
    end
    % Plot quad, fov, and estimated quad, estimated_map, and sliding window
    hold on;
            
    tR = QuatToRot(x(7:10));
    tpoint1 = tR*[params.arm_length;0;0];
    tpoint2 = tR*[0;params.arm_length;0];
    tpoint3 = tR*[-params.arm_length;0;0];
    tpoint4 = tR*[0;-params.arm_length;0];
    tproppts = params.propr*tR*[cos(propangs);sin(propangs);zeros(1,nprop)];        
    twp1 = x(1:3) + tpoint1;
    twp2 = x(1:3) + tpoint2;
    twp3 = x(1:3) + tpoint3;
    twp4 = x(1:3) + tpoint4;
    tprop1 = tproppts + twp1*ones(1,nprop);
    tprop2 = tproppts + twp2*ones(1,nprop);
    tprop3 = tproppts + twp3*ones(1,nprop);
    tprop4 = tproppts + twp4*ones(1,nprop);    
    tfov0 = x(1:3);        
    tfov1 = tR * [params.fov;  params.fov * tan(ifov*pi/180/2);  params.fov * tan(ifov*pi/180/2)] + x(1:3);
    tfov2 = tR * [params.fov;  params.fov * tan(ifov*pi/180/2); -params.fov * tan(ifov*pi/180/2)] + x(1:3);
    tfov3 = tR * [params.fov; -params.fov * tan(ifov*pi/180/2); -params.fov * tan(ifov*pi/180/2)] + x(1:3);
    tfov4 = tR * [params.fov; -params.fov * tan(ifov*pi/180/2);  params.fov * tan(ifov*pi/180/2)] + x(1:3);
    eR = RPYtoRot_ZYX(qd.roll_des,qd.pitch_des,qd.yaw_des);
    epoint1 = eR*[params.arm_length;0;0];
    epoint2 = eR*[0;params.arm_length;0];
    epoint3 = eR*[-params.arm_length;0;0];
    epoint4 = eR*[0;-params.arm_length;0];
    epoint5 = eR*[0;0;0.02];
    eproppts = params.propr*eR*[cos(propangs);sin(propangs);zeros(1,nprop)];        
    ewp1 = qd.pos_des + epoint1;
    ewp2 = qd.pos_des + epoint2;
    ewp3 = qd.pos_des + epoint3;
    ewp4 = qd.pos_des + epoint4;
    ewp5 = qd.pos_des + epoint5;
    eprop1  = eproppts + ewp1*ones(1,nprop);
    eprop2  = eproppts + ewp2*ones(1,nprop);
    eprop3  = eproppts + ewp3*ones(1,nprop);
    eprop4  = eproppts + ewp4*ones(1,nprop);     
%     emap    = [0;0;0];
%     ewindow = [0;0;0];
    if ~vis_init
        thtraj = plot3(x(1), x(2), x(3), 'b-','LineWidth',3);                        
        tharm1 = line([twp1(1),twp3(1)],[twp1(2),twp3(2)],[twp1(3),twp3(3)],'Color','b');
        tharm2 = line([twp2(1),twp4(1)],[twp2(2),twp4(2)],[twp2(3),twp4(3)],'Color','b');
        thprop1 = plot3(tprop1(1,:),tprop1(2,:),tprop1(3,:),'r-');
        thprop2 = plot3(tprop2(1,:),tprop2(2,:),tprop2(3,:),'b-');
        thprop3 = plot3(tprop3(1,:),tprop3(2,:),tprop3(3,:),'b-');
        thprop4 = plot3(tprop4(1,:),tprop4(2,:),tprop4(3,:),'b-');
        thfov1  = line([tfov0(1) tfov1(1) tfov2(1)], [tfov0(2) tfov1(2) tfov2(2)], [tfov0(3) tfov1(3) tfov2(3)],'Color','k');
        thfov2  = line([tfov0(1) tfov2(1) tfov3(1)], [tfov0(2) tfov2(2) tfov3(2)], [tfov0(3) tfov2(3) tfov3(3)],'Color','k');
        thfov3  = line([tfov0(1) tfov3(1) tfov4(1)], [tfov0(2) tfov3(2) tfov4(2)], [tfov0(3) tfov3(3) tfov4(3)],'Color','k');
        thfov4  = line([tfov0(1) tfov4(1) tfov1(1)], [tfov0(2) tfov4(2) tfov1(2)], [tfov0(3) tfov4(3) tfov1(3)],'Color','r');
        ehtraj = plot3(qd.pos_des(1), qd.pos_des(2), qd.pos_des(3), 'g-','LineWidth',3);                                    
        eharm1 = line([ewp1(1),ewp3(1)],[ewp1(2),ewp3(2)],[ewp1(3),ewp3(3)],'Color','g');
        eharm2 = line([ewp2(1),ewp4(1)],[ewp2(2),ewp4(2)],[ewp2(3),ewp4(3)],'Color','g');
        ehhat = line([qd.pos_des(1),ewp5(1)],[qd.pos_des(2),ewp5(2)],[qd.pos_des(3),ewp5(3)],'Color','k');
        ehprop1  = plot3(eprop1(1,:),eprop1(2,:),eprop1(3,:),'m-');
        ehprop2  = plot3(eprop2(1,:),eprop2(2,:),eprop2(3,:),'g-');
        ehprop3  = plot3(eprop3(1,:),eprop3(2,:),eprop3(3,:),'g-');
        ehprop4  = plot3(eprop4(1,:),eprop4(2,:),eprop4(3,:),'g-');        
%         ehmap    = plot3(emap(1,:), emap(2,:), emap(3,:),'ko','MarkerSize',10,'MarkerFaceColor','k');
%         ehwindow = plot3(ewindow(1,:), ewindow(2,:), ewindow(3,:),'ro','MarkerSize',5,'MarkerFaceColor','r');            
    else
        set(thtraj, 'XData', [get(thtraj, 'XData') x(1)]);             
        set(thtraj, 'YData', [get(thtraj, 'YData') x(2)]);                        
        set(thtraj, 'ZData', [get(thtraj, 'ZData') x(3)]);                                    
        set(thprop1,'XData',tprop1(1,:));
        set(thprop1,'YData',tprop1(2,:));
        set(thprop1,'ZData',tprop1(3,:));
        set(thprop2,'XData',tprop2(1,:));
        set(thprop2,'YData',tprop2(2,:));
        set(thprop2,'ZData',tprop2(3,:));
        set(thprop3,'XData',tprop3(1,:));
        set(thprop3,'YData',tprop3(2,:));
        set(thprop3,'ZData',tprop3(3,:));
        set(thprop4,'XData',tprop4(1,:));
        set(thprop4,'YData',tprop4(2,:));
        set(thprop4,'ZData',tprop4(3,:));
        set(tharm1,'XData',[twp1(1),twp3(1)]);
        set(tharm1,'YData',[twp1(2),twp3(2)]);
        set(tharm1,'ZData',[twp1(3),twp3(3)]);
        set(tharm2,'XData',[twp2(1),twp4(1)]);
        set(tharm2,'YData',[twp2(2),twp4(2)]);
        set(tharm2,'ZData',[twp2(3),twp4(3)]);   
        set(thfov1,'XData',[tfov0(1) tfov1(1) tfov2(1)]);
        set(thfov1,'YData',[tfov0(2) tfov1(2) tfov2(2)]);
        set(thfov1,'ZData',[tfov0(3) tfov1(3) tfov2(3)]);       
        set(thfov2,'XData',[tfov0(1) tfov2(1) tfov3(1)]);
        set(thfov2,'YData',[tfov0(2) tfov2(2) tfov3(2)]);
        set(thfov2,'ZData',[tfov0(3) tfov2(3) tfov3(3)]);   
        set(thfov3,'XData',[tfov0(1) tfov3(1) tfov4(1)]);
        set(thfov3,'YData',[tfov0(2) tfov3(2) tfov4(2)]);
        set(thfov3,'ZData',[tfov0(3) tfov3(3) tfov4(3)]);   
        set(thfov4,'XData',[tfov0(1) tfov4(1) tfov1(1)]);
        set(thfov4,'YData',[tfov0(2) tfov4(2) tfov1(2)]);
        set(thfov4,'ZData',[tfov0(3) tfov4(3) tfov1(3)]);               
        set(ehtraj, 'XData', [get(ehtraj, 'XData') qd.pos_des(1)]);
        set(ehtraj, 'YData', [get(ehtraj, 'YData') qd.pos_des(2)]);
        set(ehtraj, 'ZData', [get(ehtraj, 'ZData') qd.pos_des(3)]);
        set(ehprop1,'XData',eprop1(1,:));
        set(ehprop1,'YData',eprop1(2,:));
        set(ehprop1,'ZData',eprop1(3,:));
        set(ehprop2,'XData',eprop2(1,:));
        set(ehprop2,'YData',eprop2(2,:));
        set(ehprop2,'ZData',eprop2(3,:));
        set(ehprop3,'XData',eprop3(1,:));
        set(ehprop3,'YData',eprop3(2,:));
        set(ehprop3,'ZData',eprop3(3,:));
        set(ehprop4,'XData',eprop4(1,:));
        set(ehprop4,'YData',eprop4(2,:));
        set(ehprop4,'ZData',eprop4(3,:));
        set(eharm1,'XData',[ewp1(1),ewp3(1)]);
        set(eharm1,'YData',[ewp1(2),ewp3(2)]);
        set(eharm1,'ZData',[ewp1(3),ewp3(3)]);
        set(eharm2,'XData',[ewp2(1),ewp4(1)]);
        set(eharm2,'YData',[ewp2(2),ewp4(2)]);
        set(eharm2,'ZData',[ewp2(3),ewp4(3)]);
        set(ehhat,'XData',[qd.pos_des(1),ewp5(1)]);
        set(ehhat,'YData',[qd.pos_des(2),ewp5(2)]);
        set(ehhat,'ZData',[qd.pos_des(3),ewp5(3)]);
%         set(ehmap,'XData',emap(1,:));
%         set(ehmap,'YData',emap(2,:));
%         set(ehmap,'ZData',emap(3,:));      
%         set(ehwindow,'XData',ewindow(1,:));
%         set(ehwindow,'YData',ewindow(2,:));
%         set(ehwindow,'ZData',ewindow(3,:));                       
    end
    hold off;
    % Render
    drawnow;        
    vis_time = time;             
    vis_init = true;
    
    t=toc
    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end
end
%把后面的0去掉
xtraj = xtraj(1:iter*nstep,:);
ttraj = ttraj(1:iter*nstep);
qd.time_list = qd.time_list(1:qd.k);
qd.state_des_list = qd.state_des_list(1:qd.k,:);
qd.state_list = qd.state_list(1:qd.k,:);

%position
figure;
subplot(311);
plot(qd.time_list,qd.state_list(:,1),'b-.','linewidth',1.5);
hold on; grid on;
plot(qd.time_list,qd.state_des_list(:,1),'r','linewidth',1.5);
xlabel('Time (sec)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
ylabel('Position x (m)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
set(gca,'FontSize',11,'FontWeight','bold','FontName','Times New Roman');
h=legend('x', 'x_d');
set(h,'FontSize',11,'FontName','Times New Roman');

subplot(312);
plot(qd.time_list,qd.state_list(:,2),'b-.','linewidth',1.5);
hold on; grid on;
plot(qd.time_list,qd.state_des_list(:,2),'r','linewidth',1.5);
xlabel('Time (sec)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
ylabel('Position y (m)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
set(gca,'FontSize',11,'FontWeight','bold','FontName','Times New Roman');
h=legend('y', 'y_d');
set(h,'FontSize',11,'FontName','Times New Roman');

subplot(313);
plot(qd.time_list,qd.state_list(:,3),'b-.','linewidth',1.5);
hold on; grid on;
plot(qd.time_list,qd.state_des_list(:,3),'r','linewidth',1.5);
xlabel('Time (sec)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
ylabel('Position z (m)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
set(gca,'FontSize',11,'FontWeight','bold','FontName','Times New Roman');
h=legend('z', 'z_d');
set(h,'FontSize',11,'FontName','Times New Roman');

%velocity
figure;
subplot(311);
plot(qd.time_list,qd.state_list(:,4),'b-.','linewidth',1.5);
hold on; grid on;
plot(qd.time_list,qd.state_des_list(:,4),'g','linewidth',1.5);
xlabel('Time (sec)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
ylabel('Velocity x (m/s)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
set(gca,'FontSize',11,'FontWeight','bold','FontName','Times New Roman');
h=legend('vx', 'vx_d');
set(h,'FontSize',11,'FontName','Times New Roman');

subplot(312);
plot(qd.time_list,qd.state_list(:,5),'b-.','linewidth',1.5);
hold on; grid on;
plot(qd.time_list,qd.state_des_list(:,5),'g','linewidth',1.5);
xlabel('Time (sec)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
ylabel('Velocity y (m/s)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
set(gca,'FontSize',11,'FontWeight','bold','FontName','Times New Roman');
h=legend('vy', 'vy_d');
set(h,'FontSize',11,'FontName','Times New Roman');

subplot(313);
plot(qd.time_list,qd.state_list(:,6),'b-.','linewidth',1.5);
hold on; grid on;
plot(qd.time_list,qd.state_des_list(:,6),'g','linewidth',1.5);
xlabel('Time (sec)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
ylabel('Velocity z (m/s)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
set(gca,'FontSize',11,'FontWeight','bold','FontName','Times New Roman');
h=legend('vz', 'vz_d');
set(h,'FontSize',11,'FontName','Times New Roman');

% %acc
% figure(4);
% subplot(311);
% % plot(qd.time_list,qd.state_list(:,4),'b-.','linewidth',1.5);
% % hold on; grid on;
% plot(qd.time_list,qd.state_des_list(:,7),'g','linewidth',1.5);
% xlabel('Time (sec)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
% ylabel('Velocity x (m/s)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
% set(gca,'FontSize',11,'FontWeight','bold','FontName','Times New Roman');
% h=legend('vx', 'vx_d');
% set(h,'FontSize',11,'FontName','Times New Roman');
% 
% subplot(312);
% % plot(qd.time_list,qd.state_list(:,5),'b-.','linewidth',1.5);
% % hold on; grid on;
% plot(qd.time_list,qd.state_des_list(:,8),'g','linewidth',1.5);
% xlabel('Time (sec)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
% ylabel('Acc y (m/s)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
% set(gca,'FontSize',11,'FontWeight','bold','FontName','Times New Roman');
% h=legend('vy', 'vy_d');
% set(h,'FontSize',11,'FontName','Times New Roman');
% 
% subplot(313);
% % plot(qd.time_list,qd.state_list(:,6),'b-.','linewidth',1.5);
% % hold on; grid on;
% plot(qd.time_list,qd.state_des_list(:,9),'g','linewidth',1.5);
% xlabel('Time (sec)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
% ylabel('Velocity z (m/s)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
% set(gca,'FontSize',11,'FontWeight','bold','FontName','Times New Roman');
% h=legend('vz', 'vz_d');
% set(h,'FontSize',11,'FontName','Times New Roman');

%attitude
figure;
subplot(311);
plot(qd.time_list,qd.state_list(:,14)*180/pi,'b-.','linewidth',1.5);
hold on; grid on;
plot(qd.time_list,qd.state_des_list(:,10)*180/pi,'g','linewidth',1.5);
xlabel('Time (sec)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
ylabel('Roll (deg)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
set(gca,'FontSize',11,'FontWeight','bold','FontName','Times New Roman');
h=legend('\phi', '\phi_d');
set(h,'FontSize',11,'FontName','Times New Roman');

subplot(312);
plot(qd.time_list,qd.state_list(:,15)*180/pi,'b-.','linewidth',1.5);
hold on; grid on;
plot(qd.time_list,qd.state_des_list(:,11)*180/pi,'g','linewidth',1.5);
xlabel('Time (sec)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
ylabel('Pitch (deg)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
set(gca,'FontSize',11,'FontWeight','bold','FontName','Times New Roman');
h=legend('\theta', '\theta_d');
set(h,'FontSize',11,'FontName','Times New Roman');

subplot(313);
plot(qd.time_list,qd.state_list(:,16)*180/pi,'b-.','linewidth',1.5);
hold on; grid on;
plot(qd.time_list,qd.state_des_list(:,12)*180/pi,'g','linewidth',1.5);
xlabel('Time (sec)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
ylabel('Yaw (deg)','FontName','Times New Roman','FontSize',11,'FontWeight','bold');
set(gca,'FontSize',11,'FontWeight','bold','FontName','Times New Roman');
h=legend('\psi', '\psi_d');
set(h,'FontSize',11,'FontName','Times New Roman');


















































