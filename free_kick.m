%% Free kick
% Simulation and animation of a free kick. 
%
% Reference:
%
% Cook, B.G. and Goff, J.E., 2006. Parameter space for successful soccer
% kicks. European journal of physics, 27(4), p.865.
%
%%
%

clear ; close all ; clc

%% Parameters

% Goal position from kick position (0,0)
goal_lon_pos    = 20;   % Goal longitudinal position    [m]
goal_lat_pos    = -5;   % Goal lateral position         [m]

% Initial conditions
% Ball position [x y z]
ball_position   = [0 0 0];
% Ball velocity
ball_speed      = 27;   % Ball speed                    [m/s]
theta           = 15;   % Polar/elevation angle         [deg]
phi             = -25;  % Azimuthal angle               [deg]
% Ball initial velocity components x, y and z
vx = ball_speed*cos(theta*pi/180)*cos(phi*pi/180);
vy = ball_speed*cos(theta*pi/180)*sin(phi*pi/180);
vz = ball_speed*sin(theta*pi/180);
% Ball velocity array
ball_velocity = [vx vy vz];
% Initial conditions
states0 = [ball_position ball_velocity];

% Video
playback_speed = 0.1;                  % Speed of playback
tF      = 3;                            % Final time                    [s]
fR      = 30/playback_speed;            % Frame rate                    [fps]
dt      = 1/fR;                         % Time resolution               [s]
time    = linspace(0,tF,tF*fR);         % Time                          [s]

%% Simulation

options = odeset('Events',@(t,y) ball_floor_or_end(t,y,goal_lon_pos),'RelTol',1e-6);
[tout,yout] = ode45(@ball_dynamics,time,states0,options);

% Retrieving states
x   = yout(:,1);
y   = yout(:,2);
z   = yout(:,3);
dx  = yout(:,4);
dy  = yout(:,5);
dz  = yout(:,6);

%% Animation

figure
% set(gcf,'Position',[50 50 1280 720])  % YouTube: 720p
% set(gcf,'Position',[50 50 854 480])   % YouTube: 480p
set(gcf,'Position',[50 50 640 640])     % Social

% Create and open video writer object
v = VideoWriter('free_kick.mp4','MPEG-4');
v.Quality   = 100;
% v.FrameRate = fR;
open(v);

for i=1:length(tout)
    subplot(4,4,1:8)
    cla
    hold on ; grid on ; axis equal
    set(gca,'FontName','Verdana','FontSize',12)
    plot_field([-206.4874  199.9462   79.6781],goal_lon_pos,goal_lat_pos)
    plot3(x(1:i),y(1:i),z(1:i),'r','LineWidth',2)
    plot3(x(i),y(i),z(i),'bo','MarkerFaceColor','b','MarkerSize',4)
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    title(strcat('Free Kick - Time=',num2str(tout(i),'%.3f'),' s (Playback speed=',num2str(playback_speed),')'))
    
    subplot(4,4,9:10)
    cla
    hold on ; grid on ; axis equal
    set(gca,'FontName','Verdana','FontSize',12)
    plot_field([7.5000 -311.1862    2.5000],goal_lon_pos,goal_lat_pos)
    plot3(x(1:i),y(1:i),z(1:i),'r','LineWidth',2)
    plot3(x(i),y(i),z(i),'bo','MarkerFaceColor','b','MarkerSize',4)
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    
    subplot(4,4,13:14)
    cla
    hold on ; grid on ; axis equal
    set(gca,'FontName','Verdana','FontSize',12)
    plot_field([313.6862   -5.0000    2.5000],goal_lon_pos,goal_lat_pos)
    plot3(x(1:i),y(1:i),z(1:i),'r','LineWidth',2)
    plot3(x(i),y(i),z(i),'bo','MarkerFaceColor','b','MarkerSize',4)
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    
    subplot(4,4,[11 12 15 16])
    cla
    hold on ; grid on ; axis equal
    set(gca,'FontName','Verdana','FontSize',12)
    plot_field([7.5000   -5.0000  308.6862],goal_lon_pos,goal_lat_pos)
    plot3(x(1:i),y(1:i),z(1:i),'r','LineWidth',2)
    plot3(x(i),y(i),z(i),'bo','MarkerFaceColor','b','MarkerSize',4)
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    
    frame = getframe(gcf);
    writeVideo(v,frame);
end

close(v);

function plot_field(camera_position,goal_lon_pos,goal_lat_pos)

    % Goal
    goal_width      = 7.32; % [m]
    goal_height     = 2.44; % [m]
    % Penalty area
    penalty_area_width  = 40.2; % [m]
    penalty_area_length = 16.5; % [m]
    % Goal area
    goal_area_width  = 18.3;    %[m]
    goal_area_length = 5.5;     %[m]
    % Penalty mark
    penalty_mark_distance_from_goal = 11; %[m]
    % Penalty arc
    penalty_arc_radius      = 9.15;
%     penalty_arc_center_x    = goal_lon_pos-penalty_mark_distance_from_goal;
%     penalty_arc_center_y    = 0;
    theta_amp = acos((penalty_area_length-penalty_mark_distance_from_goal)/penalty_arc_radius);
    theta = linspace(-theta_amp+pi,theta_amp+pi,10);
    x_arc = penalty_arc_radius*cos(theta)+(goal_lon_pos-penalty_mark_distance_from_goal);
    y_arc = penalty_arc_radius*sin(theta);
    % Endline
    endline_width = 50;

    set(gca,'CameraPosition',camera_position)
    set(gca,'xlim',[-10 goal_lon_pos+5],'ylim',[-endline_width/2+goal_lat_pos endline_width/2+goal_lat_pos],'zlim',[0 5])
    % Goal
    plot3(  goal_lon_pos*ones(4,1),...
            [-goal_width/2+goal_lat_pos -goal_width/2+goal_lat_pos goal_width/2+goal_lat_pos goal_width/2+goal_lat_pos],...
            [0 goal_height goal_height 0],'k','LineWidth',2)
    % Penalty area
    plot3(  [goal_lon_pos goal_lon_pos-penalty_area_length goal_lon_pos-penalty_area_length goal_lon_pos],...
            [-penalty_area_width/2+goal_lat_pos -penalty_area_width/2+goal_lat_pos penalty_area_width/2+goal_lat_pos penalty_area_width/2+goal_lat_pos],...
            [0 0 0 0],'k')
    % Goal area
    plot3(  [goal_lon_pos goal_lon_pos-goal_area_length goal_lon_pos-goal_area_length goal_lon_pos],...
            [-goal_area_width/2+goal_lat_pos -goal_area_width/2+goal_lat_pos goal_area_width/2+goal_lat_pos goal_area_width/2+goal_lat_pos],...
            [0 0 0 0],'k')
    % Penalty mark
    plot3(goal_lon_pos-penalty_mark_distance_from_goal,goal_lat_pos,0,'ko','MarkerFaceColor','k','MarkerSize',2)
    % Penalty arc
    plot3(x_arc,y_arc+goal_lat_pos,zeros(length(theta)),'k')
    % Endline
    plot3([goal_lon_pos goal_lon_pos],[-endline_width/2+goal_lat_pos endline_width/2+goal_lat_pos],[0 0],'k')
    % Ball initial position
    plot3(0, 0, 0,'ko','MarkerFaceColor','k','MarkerSize',2)
end

function dstates = ball_dynamics(~,states)
% States
% x   = states(1);
% y   = states(2);
% z   = states(3);
dx  = states(4);
dy  = states(5);
dz  = states(6);

v = sqrt(dx^2 + dy^2 + dz^2);   % Speed                     [m/s]

% Parameters
m   = 0.425;                    % Mass                      [kg]
A   = 0.0388;                   % Cross-sectional area      [m2]
g   = 9.8;                      % Gravity                   [m/s2]
rho = 1.2;                      % Air density               [kg/m3]
r   = 0.111;                    % Radius                    [m]
CM  = 1;                        % Magnus coefficient (Magnus)
CD  = 0.275;                    % Drag coefficient (Prandtl)

% Ball angular velocity
wx = 0;
wy = 0;
wz = 6*2*pi;        % rad/s

% Lumped coefficients
C1 = 1/2*CD*rho*A;
C2 = 1/2*CM*rho*A*r;

ddx = (-C1*v*dx + C2*(wy*dz - wz*dy))/m;
ddy = (-C1*v*dy + C2*(wz*dx - wx*dz))/m;
ddz = (-C1*v*dz + C2*(wx*dy - wy*dx) - m*g)/m;

dstates = [dx ; dy ; dz ; ddx ; ddy ; ddz];

end

function [position,isterminal,direction] = ball_floor_or_end(~,y,goal_lon_pos)
% Terminate simulation when ball touches the floor or YZ plane.
position = (goal_lon_pos+5)-y(1);
isterminal = 1; 
direction = 0;   
end
