function [R_space, LW_space, errors, best_R, best_LW, min_err] = bag_error_calculation(bag_file, TEST_SIZE, R_CENTER, R_DYNAMIC, LW_CENTER, LW_DYNAMIC, T, N)
    % bag_error_calculation Computes the odometry error for the give bag and
    % the specified range of R and LW values.
    %   Parameters:
    %       bag_file: m file with the wheels velocities and optitrack data
    %       TEST_SIZE: Test size, it is actually the side length of the output
    %       error matrix
    %       R_CENTER:   Robot wheels diameter center value
    %       R_DYNAMIC:  Robot wheels diameter values side length
    %       LW_CENTER:  Robot width plus length center value
    %       LW_DYNAMIC: Robot width plus length values side length
    %       T:  Gear ratio
    %       N:  Encoder counts
    data = load(bag_file);
    wheels = data.wheels;
    optitrack = data.optitrack;

    % Downsample optitrack data
    X_opti = [optitrack.x, optitrack.y, optitrack.theta]';
    X_opti = resample(X_opti, size(wheels.time, 1), size(optitrack.time, 1), 'Dimension', 2);

    % Run error calculation
    [R_space, LW_space, errors] = error_calculation(TEST_SIZE, R_CENTER, R_DYNAMIC, LW_CENTER, LW_DYNAMIC, T, N, wheels, X_opti);

    % Find the best values
    [tmp, i] = min(errors);
    [min_err, j] = min(tmp);
    i = i(j);
    best_R = R_space(i);
    best_LW = LW_space(j);
end
