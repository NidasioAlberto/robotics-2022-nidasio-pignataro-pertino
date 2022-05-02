function [R_space, LW_space, errors] = error_calculation(TEST_SIZE, R_CENTER, R_DYNAMIC, LW_CENTER, LW_DYNAMIC, wheels, X_opti)
    robot_parameters
    
    R_space = linspace(R_CENTER - R_DYNAMIC / 2, R_CENTER + R_DYNAMIC / 2, TEST_SIZE);
    LW_space = linspace(LW_CENTER - LW_DYNAMIC / 2, LW_CENTER + LW_DYNAMIC / 2, TEST_SIZE);
    errors = zeros(TEST_SIZE, TEST_SIZE);
    
    % figure
    % hold on
    for i = 1:TEST_SIZE
        for j = 1:TEST_SIZE
            errors(i, j) = odometry_error(R_space(i), LW_space(j), T, N, wheels, X_opti);
        end
        clc
        fprintf('Progress: %d%%\n', round(i / TEST_SIZE * 100));
    end
end