clc;
clear;
close all;

addpath("supportingNR"); addpath("pathmexa64");

% Declare global variables
global h; global num_dof; global safe_dist; global links;
global obs_centers; global obs_radii;
global q_array;     global vc_array;

q_array = [];vc_array = [];

% Input set
initial_st = [0.2, 0.3, 0.3, 0.2, -0.4, -0.2, -0.2, -0.2];
goal_st = [2.7, -0.4, -0.8, -0.5, -0.2, 0.4, 0.7, 0.8];

% Define the robot parameters
links = 0.25*ones(1,length(initial_st));

% Define walls or boundary
boundary.x_min = -1.0;  boundary.x_max = 2.0;
boundary.y_min = -0.50; boundary.y_max = 2.0;

% Define obstacles
center_c1 = [0.81; 1.4];  radius1 = 0.55;
center_c2 = [-0.6; 1.0]; radius2 = 0.35;
center_c3 = [1.1; 0.1];  radius3 = 0.4;
center_c4 = [0.0; 0.3];  radius4 = 0.2;

obs_centers = [center_c1, center_c2, center_c3, center_c4];
obs_radii = [radius1, radius2, radius3, radius4];

% Other parameters
h = 0.010; num_dof = 8; safe_dist = 0.005;

% Initial and goal states
[ee_start(1), ee_start(2)] = frdNR(links, initial_st);
[ee_goal(1), ee_goal(2)] = frdNR(links, goal_st);

% Tree building parameters
max_iter = 15000;             % Maximum number of iterations
tol = 1e-4;                   % closeness to goal
each_step1 = 0.005;
each_step2 = 0.001;

% ///////////////////////////////////////////////
%             Start of the Algorithm           //
%////////////////////////////////////////////////
figure(1);
hold on;
plot(ee_start(1), ee_start(2), "ro", "MarkerSize", 10, "markerfacecolor", [1, 0, 0]);
plot(ee_goal(1), ee_goal(2), "go", "MarkerSize", 10, "markerfacecolor", [0, 1, 0]);

% Plot Wall
plot([boundary.x_min, boundary.x_min], [boundary.y_min, boundary.y_max], "K", "LineWidth", 4);
plot([boundary.x_min, boundary.x_max], [boundary.y_min, boundary.y_min], "K", "LineWidth", 4);
plot([boundary.x_max, boundary.x_max], [boundary.y_min, boundary.y_max], "K", "LineWidth", 4);
plot([boundary.x_min, boundary.x_max], [boundary.y_max, boundary.y_max], "K", "LineWidth", 4);

filledCircle(center_c1, radius1, 100, 'k');
filledCircle(center_c2, radius2, 100, 'k');
filledCircle(center_c3, radius3, 100, 'k');
filledCircle(center_c4, radius4, 100, 'k');

xlabel("x [m]"); ylabel("y [m]");
title("NR Robot Pose"); grid on;
xlim([boundary.x_min, boundary.x_max]);
ylim([boundary.y_min, boundary.y_max]);
axis("equal");

total_dst = norm(ee_goal - ee_start);   % heuristic cost

% Start building tree
reached_flag = 0;
num_iter = 0;
ee_current = ee_start;
th_current = initial_st;

while (num_iter <= max_iter) && (reached_flag == 0)   
    
    % Get taskspace and corresponding jointspace velocities
    taskspace_current_dist = norm(ee_goal - ee_current);
    ee_old = ee_current;
    if taskspace_current_dist > 10*tol
        ip_vel_task = each_step1*(ee_goal - ee_current)/taskspace_current_dist;
    else
        ip_vel_task = each_step2*(ee_goal - ee_current)/taskspace_current_dist;
    end
    
    ip_vel_joint = pinv(jacobNR(links, th_current))*ip_vel_task';

    % Call the local_planner to generate new_node
    th_new = taskspace_local_NRplanner(th_current, ip_vel_joint', num_dof);
    
    % Set th_new to th_current
    th_current = th_new;
    [ee_current(1), ee_current(2)] = frdNR(links, th_current);
    
    num_iter = num_iter + 1;
    
    % Find current distance to goal
    ee_dist_2_goal = norm(ee_goal - ee_current); % distance from th_new to goal_node
    
    % Visualize        
    visualizeNR(links, th_new);
    
    % Check if reached
    if ee_dist_2_goal < tol
        reached_flag = 1;
    end
    
    curr_move_dist = norm(ee_old - ee_current);

end

