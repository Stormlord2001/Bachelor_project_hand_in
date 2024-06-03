%% Setup docment
clc;
clear;

%% Load in data
data_opti = readtable('Data\OptiTrack\PID_with_Kalmann_test_001.csv', 'VariableNamingRule', 'preserve');
data_drone = readtable('Data\Drone\PID.csv', 'VariableNamingRule', 'preserve');

x_opti = table2array(data_opti(:, "Rigid Body_5"));
y_opti = table2array(data_opti(:, "Rigid Body_3"));
z_opti = table2array(data_opti(:, "Rigid Body_4"));

x_opti = x_opti(600:end);
y_opti = y_opti(600:end);
z_opti = z_opti(600:end);

figure(1);
title('Test of PID controllers');
subplot(3,1,1); 
hold on;
plot(-y_opti/1000)
title('Drone x-axis');
xlabel('frame #');
ylabel('World X (m)');
xline(150);
xline(520);
xline(956);
xlim([0 1350]);
hold off;

subplot(3,1,2);
hold on;
plot(x_opti/1000);
title('Drone y-axis');
xlabel('frame #');
ylabel('World Z (m)');
xline(150);
xline(520);
xline(956);
xlim([0 1350]);
hold off;

subplot(3,1,3);
hold on;
plot(z_opti/1000);
title('Drone z-axis');
xlabel('frame #');
ylabel('World Y (m)');
xline(150);
xline(520);
xline(956);
xlim([0 1350]);
hold off;