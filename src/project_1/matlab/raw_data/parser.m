parse(1)
parse(2)
parse(3)

function [] = parse(bag_number)
    raw_wheels = readtable(sprintf('bag%d_wheel_states.csv', bag_number));
    raw_robot = readtable(sprintf('bag%d_robot_pose.csv', bag_number));
    
    % Wheel states
    wheels.time = (raw_wheels.field_header_stamp - raw_wheels.field_header_stamp(1)) / 1e9;
    wheels.front_left_pos  = raw_wheels.field_position0;
    wheels.front_right_pos = raw_wheels.field_position1;
    wheels.rear_left_pos   = raw_wheels.field_position2;
    wheels.rear_right_pos  = raw_wheels.field_position3;
    wheels.front_left_speed  = raw_wheels.field_velocity0;
    wheels.front_right_speed = raw_wheels.field_velocity1;
    wheels.rear_left_speed   = raw_wheels.field_velocity2;
    wheels.rear_right_speed  = raw_wheels.field_velocity3;
    
    % Optitrack
    optitrack.time = (raw_robot.field_header_stamp - raw_robot.field_header_stamp(1)) / 1e9;
    optitrack.x = raw_robot.field_pose_position_x;
    optitrack.y = raw_robot.field_pose_position_y;
    eul_orientation = quat2eul([ ...
        raw_robot.field_pose_orientation_w, ...
        raw_robot.field_pose_orientation_x, ...
        raw_robot.field_pose_orientation_y, ....
        raw_robot.field_pose_orientation_z,
    ]);
    optitrack.theta = unwrap(eul_orientation(:, 1));
    
    save(sprintf('../parsed_data/bag%d', bag_number), "optitrack", "wheels")
end