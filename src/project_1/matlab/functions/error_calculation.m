function [R_space, LW_space, errors] = error_calculation(TEST_SIZE, R_CENTER, R_DYNAMIC, LW_CENTER, LW_DYNAMIC, T, N, wheels, X_opti)
    % error_calculation Computes the odometry error for the specified range of
    % R and LW values.
    %   Parameters:
    %       TEST_SIZE: Test size, it is actually the side length of the output
    %       error matrix
    %       R_CENTER:   Robot wheels diameter center value
    %       R_DYNAMIC:  Robot wheels diameter values side length
    %       LW_CENTER:  Robot width plus length center value
    %       LW_DYNAMIC: Robot width plus length values side length
    %       T:  Gear ratio
    %       N:  Encoder counts
    %       wheels: Wheels velocities array [4x...]
    %       X_opti: Reference positions
    %
    %   The functions creates two arrays of R and LW values with a size of
    %   TEST_SIZE and values ranging from CENTER - DYNAMIC / 2 to CENTER + DYNAMIC / 2
    %   and then calculates the odometry error with every possible combination
    %   of the parameters.

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
