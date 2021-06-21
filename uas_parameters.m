%% INITIALIZATION
clear
close all
clc
%% Setup the map
% Load the map from the wall file. Each line, except the two last, in the
% wall file is an obstacle. The position of the obstacle is defined in the
% wall file by its x y z coordinates. The three last elements is
% the size of the maze, the starting position and goal position
load('auto_wall.txt')
% If a different named file is used, then write it into the wall variable
% e.g. wall = maze_2;
wall = auto_wall;
% Define the map size
max_x = wall(length(wall) - 2, 1);
max_y = wall(length(wall) - 2, 2);
max_z = wall(length(wall) - 2, 3);
map = zeros(max_x, max_y, max_z);
map = zeros(max_x, max_y, max_z);
% Input the obstacles into the map
for i = 1:(length(wall) - 3)
    map = gen_square3d([wall(i, 1) wall(i, 1) + 1;...
                        wall(i, 2) wall(i, 2) + 1;...
                        wall(i, 3) wall(i, 3) + 1], map);
end
%% SIMULATION PARAMETERS

[route,start,end_]=astar_3d1(map,'H','A');
route
%makes sure end and start is not 1
map(start(1), start(2), start(3)) = 0;
map(end_(1), end_(2), end_(3)) = 0;
wall_color = [0.8 0.2 0.2];
sample_time = 4e-2;
publish_rate = 1 * sample_time;
x0 = 36;
y0 = 80;
z0 = 1;
g = 9.80665 ;
mass_drone = 0.68 ;
mass_rod = 0.0;
mass_tip = 0;
mass_total = mass_drone + mass_rod + mass_tip;
stiffness_rod = 100 ;
critical_damping_rod = 2 * sqrt(mass_total * stiffness_rod) ;
stiffness_wall = 100 ;
critical_damping_wall = 2 * sqrt(mass_total * stiffness_wall) ;
inertia_xx = 0.007 ;
inertia_yy = 0.007 ;
inertia_zz = 0.012 ;
arm_length = 0.17 ;
rotor_offset_top = 0.01 ;
motor_constant = 8.54858e-06 ;
moment_constant = 0.016 ;
max_rot_velocity = 838 ;
allocation_matrix = ...
    [1 1 1 1
     0 arm_length 0 -arm_length
     -arm_length 0 arm_length 0
     -moment_constant moment_constant -moment_constant moment_constant] ;
mix_matrix = inv(motor_constant * allocation_matrix) ;
air_density = 1.2041;
drag_coefficient = 0.47;
reference_area = pi * 75e-3^2;

%% functions 
function [ route,start,end_ ] = astar_3d1( map, start_char, end_char)
    % Define the limits of the map
    if start_char == 'E'
        start = [1,1,1];
    elseif start_char == 'F'
        start = [2,1,1];
    elseif start_char == 'G'
        start = [3,1,1];
    elseif start_char == 'H'
        start = [4,1,1];
    end
    
    if end_char == 'A'
        end_= [1,5,1];
    elseif end_char == 'B'
        end_ =[2,5,1];
    elseif end_char == 'C'
        end_ =[3,5,1];
    elseif end_char == 'D'
        end_=[4,5,1];
    end 
    
    max_x = size(map,1);
    max_y = size(map,2);
    max_z = size(map,3);
    % Children must be initalized to have nodes in it
    % The arrays keeping track of the nodes must initialized 
    % containing a node. These flags tells the first node in the
    % closed and children array to be put in directly
    first_closed = 1;
    first_children = 1;
    closed = [];
    children = [];

    % Create the first node at the start position
    parent_node = node;
    parent_node.position = start;
    parent_node.h = parent_node.calc_dist_3d(end_);
    parent_node.f = parent_node.h;

    % Flag used to skip nodes which is already added
    continue_flag = 0;

    % Slow the calculation down,
    % so it can be followed in real time
    pause on;

    % Keep running until the end point is reached
    while ~(parent_node.position(1) == end_(1) && ...
            parent_node.position(2) == end_(2) && ...
            parent_node.position(3) == end_(3))
        % Run through the surronding squares
        for x = -1:1
            for y = -1:1
                for z = -1:1
                    % Skip the node itself
                    % And also dont allow for diagonal movement
                    % As that will create problems when navigating the 
                    % real maze
                    if ~((x == 0 && y == 0 && z== 0) ||...
                         (abs(x) + abs(y)+abs(z) > 1))
                        node_pos = [parent_node.position(1) + x, ...
                                    parent_node.position(2) + y, ...
                                    parent_node.position(3) + z];
                        % Check if the children is within the map
                        if ~(node_pos(1) < 1 || node_pos(1) > max_x || ...
                             node_pos(2) < 1 || node_pos(2) > max_y || ...
                             node_pos(3) < 1 || node_pos(3) > max_z)
                            % Check if the children is an obstacle
                            if ~(map(node_pos(1),node_pos(2),node_pos(3)) == 1)
                                % Check if the node have been visited
                                for closed_i = 1:length(closed)
                                    if node_pos == closed(closed_i).position
                                        % Note that this node is not 
                                        % to be added to children
                                        continue_flag = 1;
                                    end
                                end
                                % Check if the node is already a child
                                for child_i = 1:length(children)
                                    if node_pos == children(child_i).position
                                        % Note that this node is not 
                                        % to be added to children
                                        continue_flag = 1;
                                    end
                                end
    
                                % Check if this node should be skipped
                                if continue_flag == 1
                                    continue_flag = 0;
                                    continue
                                end
    
                                % Define the child node
                                temp_node = node;
                                % Note its parent
                                temp_node.parent = parent_node;
                                % Note its position
                                temp_node.position = node_pos;
                                % Calculate the distance from the node
                                % to the end point
                                temp_node.h = temp_node.calc_dist_3d(end_);
                                temp_node.g = temp_node.calc_dist_tostart_3d(start);
                                % Calculate the total cost of the node
                                temp_node.f = temp_node.h+temp_node.g; 
    
                                % Add the node to the children array
                                % Check if it is the first child
                                % being added
                                if first_children == 1 
                                    first_children = 0;
                                    children = [temp_node]; 
                                else
                                    % Otherwise expand the children array
                                    children(end+1) = temp_node;
                                end
                            end
                        end
                    end
                end 
            end
        end

    % Add the parent node to the list of closed nodes
    if first_closed == 1
        first_closed = 0;
        closed = [parent_node]; 
    else
        closed(end+1) = parent_node;
    end
        % Choose the child node with the lowest f value
        lowest_f = 999999;
        lowest_child_i = -1;
        for child_i = 1:length(children)
            if children(child_i).f < lowest_f
                lowest_f = children(child_i).f;
                lowest_child_i = child_i;
            end
        end

        % Check if there still is routes avaliable 
        if length(children) == 0
           route = NaN;
           return
        end
        
        % Update the parent to the children
        % with the lowest f value
        parent_node = children(lowest_child_i);

        % Delete the new parent from the children
        children(lowest_child_i) = [];
    end

    % Find the route that the algorithm took
    % Init the route array
    route = [parent_node.position];
    % Keep going until the route is back at the start position
    while  ~(parent_node.position(1) == start(1) && ...
             parent_node.position(2) == start(2) && ...
             parent_node.position(3) == start(3))
       % Update the route by going backwards through the parents
       parent_node = parent_node.parent;
       route = cat(1,route,parent_node.position);
    end
    route = flip(route);
    %scale and add offests
    x_scale = 0.65;
    y_scale = 0.55;
    z_scale = 0.75;

    x_offset = 0.3;
    y_offset = 0.5;
    z_offset = 0.45;

    % Make a copy of the route
    route_scaled = route;

    % Scale the copy
    route_scaled(:,1) = (route_scaled(:,1) - 1) * x_scale + x_offset;
    route_scaled(:,2) = (route_scaled(:,2) - 1) * y_scale + y_offset;
    route_scaled(:,3) = (route_scaled(:,3) - 1) * z_scale + z_offset;

    % Print the scaled route
    route_length=length(route);
    route_scaled = cat(1,route_scaled,[route_scaled(route_length,1),route_scaled(route_length,2),0.1]);
    
    route=route_scaled;
end