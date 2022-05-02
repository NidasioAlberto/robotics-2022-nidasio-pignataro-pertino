load("parsed_data/bag3.mat")

% Downsample optitrack data
X_opti = [optitrack.x, optitrack.y, optitrack.theta]';
X_opti = resample(X_opti, size(wheels.time, 1), size(optitrack.time, 1), 'Dimension', 2);

[R_space, LW_space, errors] = error_calculation(71, 0.07534, 0.004, 0.3556, 0.02, wheels, X_opti);

figure
hold on
s = surf(LW_space, R_space, errors);
s.EdgeColor = 'none';
xlabel('R = wheels radius')
ylabel('LW = robot dimension')
zlabel('error')
% set(gca,'DataAspectRatio',[1 5 500000])
grid on

% Find the minimum
[tmp, i] = min(errors);
[minimum, j] = min(tmp);
i = i(j);
fprintf('Best configuration: LW=%f R=%f e=%f\n', LW_space(j), R_space(i), minimum);
h = scatter3(LW_space(j), R_space(i), minimum, 'filled', 'r');
h.SizeData = 100;