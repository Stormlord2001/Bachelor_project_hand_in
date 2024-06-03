clc;
clear;

%% Input data

% Choose plots
plot_full_data = 1;
plot_zoom = 0;
plot_ecdf = 1;
save_svg = 1;

remove_start_points = 200;



% Marker size
%%{ 
% Import test data
opti = readtable('OptiTrack_csv\Marker_size_test_001.csv', 'VariableNamingRule', 'preserve');
drone = readtable("drone_footage\marker_size.csv", 'VariableNamingRule','preserve');
amount_poseestimators = 7;
title_of_plots = 'Impact of marker size';
boxplot_labels = {'ID 1','ID 2','ID 3','ID 4','ID 5','ID 6','ID 7'}; 
ecdf_labels = {'ID 1','ID 2','ID 3','ID 4','ID 5','ID 6','ID 7', 'Q3', 'Median', 'Q1'};
plot_labels = {'OptiTrack', 'ID 1','ID 2','ID 3','ID 4','ID 5','ID 6','ID 7'};
location_to_save = 'Test_images/marker_size/';
%ecdf_save_names = {'ecdf_marker_size_x.svg', 'ecdf_marker_size_y.svg', 'ecdf_marker_size_z.svg', 'ecdf_marker_size_yaw.svg'};
ecdf_save_names = {'ecdf_marker_size_x.eps', 'ecdf_marker_size_y.eps', 'ecdf_marker_size_z.eps', 'ecdf_marker_size_yaw.eps'};
full_data_name = 'full_data_size.eps';
sync_drone = 0;
sync_opti = 1704-1190;
%}


% Marker amount
%{ 
% Import test data
opti = readtable('OptiTrack_csv\Marker_amount_test_001.csv', 'VariableNamingRule', 'preserve');
drone = readtable("drone_footage\marker_amount_real.csv", 'VariableNamingRule','preserve');
amount_poseestimators = 4;
title_of_plots = 'Impact of marker amount';
boxplot_labels = {'1 Marker', '2 Markers', '3 Markers', '4 Markers'};
ecdf_labels = {'1 Marker', '2 Markers', '3 Markers', '4 Markers', 'Q3', 'Median', 'Q1'};
plot_labels = {'OptiTrack', '1 Marker', '2 Markers', '3 Markers', '4 Markers'};  
location_to_save = 'Test_images/marker_amount/';
%ecdf_save_names = {'ecdf_marker_amount_x.svg', 'ecdf_marker_amount_y.svg', 'ecdf_marker_amount_z.svg', 'ecdf_marker_amount_yaw.svg'};
ecdf_save_names = {'ecdf_marker_amount_x.eps', 'ecdf_marker_amount_y.eps', 'ecdf_marker_amount_z.eps', 'ecdf_marker_amount_yaw.eps'};
full_data_name = 'full_data_amount.eps';
sync_drone = 0;
sync_opti = 783-568;
%}


% Marker angle
%{ 
% Import test data
opti = readtable('OptiTrack_csv\Marker_angle_test_001.csv', 'VariableNamingRule', 'preserve');
drone = readtable("drone_footage\marker_angle_real.csv", 'VariableNamingRule','preserve');
amount_poseestimators = 4;
title_of_plots = 'Impact of marker angle';
boxplot_labels = {'0 Degrees', '9 Degrees', '12 Degrees', '21 Degrees'}; 
ecdf_labels = {'0 Degrees', '9 Degrees', '12 Degrees', '21 Degrees', 'Q3', 'Median', 'Q1'};
plot_labels = {'OptiTrack', '0 Degrees', '9 Degrees', '12 Degrees', '21 Degrees'};
location_to_save = 'Test_images/marker_angle/';
%ecdf_save_names = {'ecdf_marker_angle_x.svg', 'ecdf_marker_angle_y.svg', 'ecdf_marker_angle_z.svg', 'ecdf_marker_angle_yaw.svg'};
ecdf_save_names = {'ecdf_marker_angle_x.eps', 'ecdf_marker_angle_y.eps', 'ecdf_marker_angle_z.eps', 'ecdf_marker_angle_yaw.eps'};
full_data_name = 'full_data_angle.eps';
sync_drone = 0;
sync_opti = 609-487;
%}

% Marker angle
%{ 
% Import test data
opti = readtable('OptiTrack_csv\Marker_angle_test_001.csv', 'VariableNamingRule', 'preserve');
drone = readtable("drone_footage\marker_angle.csv", 'VariableNamingRule','preserve');
amount_poseestimators = 4;
title_of_plots = 'Impact of Kalman Filter';
boxplot_labels = {'0 Degrees', '9 Degrees', '12 Degrees', '21.9 Degrees'}; 
ecdf_labels = {'0 Degrees', '9 Degrees', '12 Degrees', '21.9 Degrees', 'Q3', 'Median', 'Q1'};
plot_labels = {'OptiTrack', '0 Degrees', '9 Degrees', '12 Degrees', '21.9 Degrees'};
location_to_save = 'Test_images/marker_angle/';
ecdf_save_names = {'ecdf_marker_angle_x.svg', 'ecdf_marker_angle_y.svg', 'ecdf_marker_angle_z.svg', 'ecdf_marker_angle_yaw.svg'};
sync_drone = 0;
sync_opti = 0;
%}


%% Read data from csv

% For opti track system
opti_i = table2array(opti(:,1));
opti_x = table2array(opti(:,"Rigid Body_5"));
opti_y = table2array(opti(:,"Rigid Body_3"));
opti_z = table2array(opti(:,"Rigid Body_4"));
opti_yaw = table2array(opti(:,"Rigid Body_2"));

% For drone
drone_i = table2array(drone(:,"Var1"));
drone_x = zeros(size(drone, 1), ((size(drone, 2)-1)/4));
drone_y = zeros(size(drone, 1), ((size(drone, 2)-1)/4));
drone_z = zeros(size(drone, 1), ((size(drone, 2)-1)/4));
drone_yaw = zeros(size(drone, 1), ((size(drone, 2)-1)/4));

for i = 1:((size(drone, 2)-1)/4)
    drone_x(:,i) = table2array(drone(:,strcat('X', int2str(i))));
    drone_y(:,i) = table2array(drone(:,strcat('Y', int2str(i))));
    drone_z(:,i) = table2array(drone(:,strcat('Z', int2str(i))));
    drone_yaw(:,i) = table2array(drone(:,strcat('Yaw', int2str(i))));
end
drone_x(drone_x==0.0) = NaN;
drone_y(drone_y==0.0) = NaN;
drone_z(drone_z==0.0) = NaN;
drone_yaw(drone_yaw==0.0) = NaN;



%% Match up the two measurements
% Data for sync

end_value = min([size(drone_i, 1), size(opti_i, 1)]);

% OptiTrack
opti_i = [opti_i(1:end_value-remove_start_points)];
opti_x = [opti_x(sync_opti+remove_start_points+1:end_value+sync_opti)];
opti_y = [opti_y(sync_opti+remove_start_points+1:end_value+sync_opti)];
opti_z = [opti_z(sync_opti+remove_start_points+1:end_value+sync_opti)];
opti_yaw = [opti_yaw(sync_opti+remove_start_points+1:end_value+sync_opti)];

% Drone
dronesync_x = zeros(size(drone, 1)-sync_drone-remove_start_points, ((size(drone, 2)-1)/4));
dronesync_y = zeros(size(drone, 1)-sync_drone-remove_start_points, ((size(drone, 2)-1)/4));
dronesync_z = zeros(size(drone, 1)-sync_drone-remove_start_points, ((size(drone, 2)-1)/4));
dronesync_yaw = zeros(size(drone, 1)-sync_drone-remove_start_points, ((size(drone, 2)-1)/4));

drone_i = [drone_i(1:end_value-sync_drone-remove_start_points)];
for i = 1:((size(drone, 2)-1)/4)
    dronesync_x(:, i) = [drone_x(sync_drone+remove_start_points+1:end_value, i)];
    dronesync_y(:, i) = [drone_y(sync_drone+remove_start_points+1:end_value, i)];
    dronesync_z(:, i) = [drone_z(sync_drone+remove_start_points+1:end_value, i)];
    dronesync_yaw(:, i) = [drone_yaw(sync_drone+remove_start_points+1:end_value, i)];

end


%% Plot data

to_plot_drone_list = [dronesync_x, dronesync_y, dronesync_z, dronesync_yaw];
to_plot_opti_list = [opti_x/1000, opti_y/1000, opti_z/1000, deg2rad(opti_yaw)];
if plot_full_data == 1
    f = figure(1);
    f.Position(3:4) = [1080 1080];
    subplot(4,1,1)
    [t,s] = title(title_of_plots);
    t.FontSize =16;
end
x_label_list = ["Frames #", "Frames #", "Frames #", "Frames #"];
y_label_list = ["x [m]", "y [m]", "z [m]", "yaw [rad]"];



for i = 0:3
    to_plot_drone = to_plot_drone_list(:,i*amount_poseestimators+1:i*amount_poseestimators+amount_poseestimators,:);
    to_plot_opti = to_plot_opti_list(:,i+1);
    
    if plot_full_data == 1
        subplot(4,1,i+1)
        
        hold on
        plot(opti_i, to_plot_opti);
        for j = 1:size(to_plot_drone, 2)
            plot(drone_i, to_plot_drone(:,j))
        end

        
        
        legend(plot_labels, "FontSize", 12);
        xlabel(x_label_list(i+1))
        ylabel(y_label_list(i+1))
        hold off
    end
end
if plot_full_data == 1
    loc_name = string(append(location_to_save, full_data_name));
    if save_svg == 1
        %print(test, loc_name,'-dsvg');
        print(loc_name,'-depsc');
    end
end

%% Calculate error
calc_error_for = dronesync_y;
calc_for_opti = opti_y;


idx = 1;
for i = 1:size(calc_error_for, 1)
    k = 0;
    for j = 1:size(calc_error_for, 2)
        if ~isnan(calc_error_for(i,j))
            k = k + 1;
        end
    end
    if k == size(calc_error_for, 2)
        for j = 1:size(calc_error_for, 2)
            error_lists(idx,j) = calc_error_for(i,j) - calc_for_opti(i)/1000;
        end
        idx = idx + 1;
    end
end


%% Subplot with the ecdf plots 

to_plot_drone_list = [dronesync_x, dronesync_y, dronesync_z, dronesync_yaw];
to_plot_opti_list = [opti_x/1000, opti_y/1000, opti_z/1000, deg2rad(opti_yaw)];

if plot_ecdf == 1
    figure(2)
end
%subplot(3,1,1)
%[t,s] = title(title_of_plots);
%t.FontSize =16;
y_label_list = ["Percentile","Percentile","Percentile","Percentile"];
x_label_list = ["x_{error} [m]", "y_{error} [m]", "z_{error} [m]", "yaw_{error} [rad]"];

for i = 0:3
    clear error_lists
    calc_error_for = to_plot_drone_list(:,i*amount_poseestimators+1:i*amount_poseestimators+amount_poseestimators,:);
    calc_for_opti = to_plot_opti_list(:,i+1);
    
    idx = 1;
    for h = 1:size(calc_error_for, 1)
        k = 0;
        for j = 1:size(calc_error_for, 2)
            if ~isnan(calc_error_for(h,j))
                k = k + 1;
            end
        end
        if k == size(calc_error_for, 2)
            for j = 1:size(calc_error_for, 2)
                error_lists(idx,j) = calc_error_for(h,j) - calc_for_opti(h);
            end
            idx = idx + 1;
        end
    end

    if plot_ecdf == 1
        test = figure(i+2);
        %subplot(2,2,i+1)
        title(title_of_plots, "FontSize", 14);
        %boxplot(error_lists, 'labels',boxplot_labels);
        
        hold on
        for j = 1:amount_poseestimators
            sorted_error_list = sort(error_lists(:,j));
            ecdf(sorted_error_list);
            quantile_list(i*2+1:i*2+2,j) = quantile(sorted_error_list, [0.025, 0.975]);
        end
        
        yline([0.25, 0.5, 0.75])
        
        xlim([-0.5, 0.5])
        ylim([0, 1])
    
        legend(ecdf_labels, "FontSize", 14);
        ylabel(y_label_list(i+1), "FontSize", 16)
        xlabel(x_label_list(i+1), "FontSize", 16)
        hold off
    else
        for j = 1:amount_poseestimators
            sorted_error_list = sort(error_lists(:,j));
            quantile_list(i*2+1:i*2+2,j) = quantile(sorted_error_list, [0.025, 0.975]);
        end
    end
    loc_name = string(append(location_to_save, ecdf_save_names(i+1)));
    if save_svg == 1
        %print(test, loc_name,'-dsvg');
        print(loc_name,'-depsc');
    end
    mean_var_list(:,i*2+1) = transpose(mean(error_lists));
    mean_var_list(:,i*2+2) = transpose(sqrt(var(error_lists)));
end
%disp("Mean and then var for each run in each direction")
mean_var_list
transpose(quantile_list)

%% Median and 25- and 75% percentiles
%Q = transpose(quantile(error_lists, [0.05, 0.25, 0.5, 0.75, 0.95]))
if plot_zoom == 1
    to_plot_drone = dronesync_z;
    to_plot_opti = opti_z/1000;
    figure(7)
    
    hold on
    plot(opti_i, to_plot_opti);
    for j = 1:size(to_plot_drone, 2)
        plot(drone_i, to_plot_drone(:,j))
    end
    
    legend(plot_labels, "FontSize", 12);
    xlabel("Frames #", "FontSize", 12)
    ylabel("z [m]", "FontSize", 12)
    hold off
end

