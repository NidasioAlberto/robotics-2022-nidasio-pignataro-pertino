% This script tests different robot parameters over the 3 bags of data and
% computes the best possible parameters to use for odometry.

addpath('functions/')
robot_parameters;

TEST_SIZE = 71;
R_CENTER = 0.07534;
R_DYNAMIC = 0.004;
LW_CENTER = 0.3556;
LW_DYNAMIC = 0.02;

% [R_space1, LW_space1, errors1, best_R1, best_LW1, min_err1] = ...
%     bag_error_calculation('parsed_data/bag1.mat', TEST_SIZE, R_CENTER, R_DYNAMIC, LW_CENTER, LW_DYNAMIC, T, N);
% [R_space2, LW_space2, errors2, best_R2, best_LW2, min_err2] = ...
%     bag_error_calculation('parsed_data/bag2.mat', TEST_SIZE, R_CENTER, R_DYNAMIC, LW_CENTER, LW_DYNAMIC, T, N);
% [R_space3, LW_space3, errors3, best_R3, best_LW3, min_err3] = ...
%     bag_error_calculation('parsed_data/bag3.mat', TEST_SIZE, R_CENTER, R_DYNAMIC, LW_CENTER, LW_DYNAMIC, T, N);

%% Plot error data
figure
hold on

% Error
s1 = surf(LW_space1, R_space1, errors1);
s1.EdgeColor = 'none';
s2 = surf(LW_space2, R_space2, errors2);
s2.EdgeColor = 'none';
s3 = surf(LW_space3, R_space3, errors3);
s3.EdgeColor = 'none';

% Best parameters for each bag
h1 = scatter3(best_LW1, best_R1, min_err1, 'filled', 'r');
h1.SizeData = 100;
h2 = scatter3(best_LW2, best_R2, min_err2, 'filled', 'r');
h2.SizeData = 100;
h3 = scatter3(best_LW3, best_R3, min_err3, 'filled', 'r');
h3.SizeData = 100;

% Plot labels
xlabel('LW = robot dimension')
ylabel('R = wheels radius')
zlabel('error')
grid on

%% Best parameters overall
best_R = (best_R2 + best_R3) / 2;
best_LW = (best_LW2 + best_LW3) / 2;
fprintf('Best configuration: R=%f LW=%f\n', best_R, best_LW);
