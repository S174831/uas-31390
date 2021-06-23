
% % Initialization
% close all
% clear
% clc

% Trajectory generation
% start_point = [double(0), double(-2), double(0)]
temp_point_1 = p_h1-(u_h1/10)
temp_point_2 = p_h1+(u_h1/10)



knots = [0 2];
waypoints = cell(1,2);
waypoints{1} = [0;-2;0]
waypoints{2} = [0;-2;2]

% waypoints{3} = [temp_point_1(1); temp_point_1(2); temp_point_1(3)];
%waypoints{2} = [0 ; -2 ; 3];
% waypoints{4} = [temp_point_2(1); temp_point_2(2); temp_point_2(3)];
% Fix this...
order = 7;
corridors.times = [1];
corridors.x_lower = [-1];
corridors.x_upper = [1];
corridors.y_lower = [-3];
corridors.y_upper = [-1];
corridors.z_lower = [0];
corridors.z_upper = [3];
% ...until here
make_plots = true;

poly_traj = uas_minimum_snap(knots, order, waypoints, corridors, make_plots);
save('uas_poly_traj', 'poly_traj', 'knots')
