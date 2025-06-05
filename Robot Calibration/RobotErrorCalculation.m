function RobotErrorCalculation()
    % csv에서 데이터 불러오기
    data = readmatrix('Calibration_Sphere_Measurement_Test_1.xlsx', 'Range', [4, 2, 53, 12]); % 시작 셀 위치 ~ 끝 셀 위치
    disp(data)

    % Calibration Sphere의 실제 좌표 (광학테이블)
    real_sphere_coordinates = [500, 10, 75]; %Unity상 로봇 원점 Y+ 10mm 고려(광학테이블)

    % % 초기 추정값 (협업 로봇의 D-H 파라미터 입력)
    % % DH 값 .csv에서 행렬 추출
    DH_params_matrix_size = rand(1, 24);
    Number_of_position = 1;
    Number_of_measurement = size(data, 1);
    disp("측정 위치 개수:");
    disp(Number_of_position);
    disp("측정 횟수:");
    disp(Number_of_measurement);

    for i = 1:Number_of_position
        for  j = 1:Number_of_measurement
            % theta, d, a, alpha
            DH_i_j = [data(j,3), 152.5, 0, -90;
                      data(j,4)-90, 200, 411, 0; %두번째 조인트가 양수일때 X+ 방향
                      data(j,5)-90, -165.5, 0, -90;
                      data(j,6), 368, 0, 90;
                      data(j,7), 0, 0, -90;
                      data(j,8), 121, 0, 0;
                     ];
            initial_DH_params(i, j) = {DH_i_j};
            %disp(initial_DH_params{i,j})
        end
    end

    % Hand-eye Pose 추정
    initial_hand_eye_pose = [0, -32, 74.5, 0.0, 0.0, 0.0];

    % Calibration Sphere 실 측정값
    measured_radius_of_arc = data(:, 2);
    Sphere_radius = 10; %(Calibration sphere의 반지름, 20mm 구 사용)
    length_opposite_format = Sphere_radius*ones(1, size(data,1));
    for i = 1:size(data, 1)
        length_opposite(i) = sqrt(length_opposite_format(i)^2 - measured_radius_of_arc(i)^2);
    end 
    %disp(length_opposite)

    measured_distance =  data(:,1);
    for i = 1:size(data, 1)
        measured_adjacent(i) = (40 - measured_distance(i)) + 105 + measured_radius_of_arc(i);
    end
    %disp(measured_adjacent)

    for i = 1:Number_of_position
        for j  = 1:Number_of_measurement
            Measured_distance_camera_sphere(i, j) = sqrt(length_opposite(j)^2 + measured_adjacent(j)^2);
        end
    end
    disp(Measured_distance_camera_sphere)

    % 옵션 Settings: Levenberg-Marquardt
    initial_deltas = zeros(1, length(DH_params_matrix_size) + length(initial_hand_eye_pose));
    %disp(initial_deltas)
    options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt', ...
        'Display', 'iter','StepTolerance', 1e-6, 'FunctionTolerance', 1e-10, 'MaxIterations', 100000);

    lb = -100000.0 * ones(1, 30); 
    ub =  100000.0 * ones(1, 30);
    global error_log iteration_count count;
    error_log = {};        
    iteration_count = 1; 
    count = 1;

    % lsqnonlin 최적화 수행
    optimized_deltas = lsqnonlin(@(params) error_function(params, Measured_distance_camera_sphere, ...
        initial_DH_params, initial_hand_eye_pose, real_sphere_coordinates), initial_deltas, lb, ub, options);

    disp('Optimized Parameters:');
    disp(optimized_deltas);

    M = cell2mat(error_log);
    writematrix(M, 'Error_Trend.xlsx');
    disp(size(error_log,2));

end

% 오차 추정
function errors = error_function(params, Measured_distance_camera_sphere, initial_DH_params, initial_hand_eye_pose, real_sphere_coordinates)

    % D-H params
    delta_DH_params = params(1:24);
    %disp(delta_DH_params);
    current_delta_DH_params = reshape(delta_DH_params, 4, 6)';
    %disp(current_delta_DH_params);

    % Hand-eye params
    delta_Hand_eye_pose_params_Trans = params(25:27);
    delta_Hand_eye_pose_params_Rotat = params(28:30);
    Hand_eye_pose_params_Trans = initial_hand_eye_pose(1:3) + delta_Hand_eye_pose_params_Trans;
    Hand_eye_pose_params_Rotat = initial_hand_eye_pose(4:6) + delta_Hand_eye_pose_params_Rotat;

    num_points_1 = size(Measured_distance_camera_sphere, 1); % 위치 개수
    num_points_2 = size(Measured_distance_camera_sphere, 2); % 위치별 측정 건수
    Predicted_distance = zeros(num_points_1, num_points_2);

    %disp(initial_DH_params{1, 1})
    %disp(delta_DH_params)

    for i = 1:num_points_1
        for j = 1:num_points_2
            Position_Calculation = forward_kinematics(initial_DH_params{i, j} + current_delta_DH_params, ...
                Hand_eye_pose_params_Trans, Hand_eye_pose_params_Rotat);
            %disp("current pos")
            %disp(Position_Calculation);
            Predicted_length_of_camera_sphere = norm(Position_Calculation - real_sphere_coordinates(i, :));
            %disp(real_sphere_coordinates(i, :))
            %disp(Predicted_length_of_camera_sphere);
            Predicted_distance(i, j) = Predicted_length_of_camera_sphere;
        end
    end
    %disp("Measurement: ");
    %disp(Measured_distance_camera_sphere);
    %disp("Prediction: ");
    %disp(Predicted_distance);

    % 오차 계산
    errors = Measured_distance_camera_sphere - Predicted_distance;

    % Error 따로 코드 용
    %errors = errors(:);

    % Error 통합 코드 용
    %errors = mean(abs(errors));
    errors = mean(errors.^2);

    %disp('Error values:');
    %disp(errors);
    %x=robot;

    global error_log iteration_count count

    if mod(iteration_count, 50) == 0  %Error 따로=17
        % Error 따로 코드 용
        %error_log{count} = errors(:); 

        % Error 통합 코드 용
        error_log{count} = errors;
        count = count + 1;
    end
    iteration_count = iteration_count + 1;

end

% Forward kinematics
function pos = forward_kinematics(DH_params,Hand_eye_pose_params_Trans, Hand_eye_pose_params_Rotat)
    T_robot = eye(4);
    %disp(DH_params);

    for i = 1:6
        theta = DH_params(i, 1);  % 조인트 각도
        d = DH_params(i, 2);  % 링크 오프셋
        a = DH_params(i, 3);  % 링크 길이
        alpha = DH_params(i, 4);  % 링크 회전 각도
        %disp(theta)
        %disp(d)
        %disp(a)
        %disp(alpha)
        theta = mod(theta + 180, 360) - 180;
        thetar = deg2rad(theta);
        alphar = deg2rad(alpha);

        % Matrix Transformation
        A = [cosd(theta), -sind(theta)*cosd(alpha),  sind(theta)*sind(alpha), a*cosd(theta);
             sind(theta),  cosd(theta)*cosd(alpha), -cosd(theta)*sind(alpha), a*sind(theta);
             0,           sind(alpha),             cosd(alpha),            d;
             0,           0,                      0,                     1];

        RA = [cos(thetar), -sin(thetar)*cos(alphar),  sin(thetar)*sin(alphar), a*cos(thetar);
             sin(thetar),  cos(thetar)*cos(alphar), -cos(thetar)*sin(alphar), a*sin(thetar);
             0,           sin(alphar),             cos(alphar),            d;
             0,           0,                      0,                     1];

        %disp(A)
        T_robot = T_robot * A;
        %disp(T_robot)
        %disp(T_robot(1:3, 4)')
    end

    % Hand-eye 회전 행렬 추출
    % Transformation 방식
    T_HE = eye(4);
    R = eul2rotm(Hand_eye_pose_params_Rotat * pi/180);
    %disp(Hand_eye_pose_params_Rotat);
    %disp(Hand_eye_pose_params_Trans);
    T_HE(1:3, 1:3) = R;
    T_HE(1:3, 4) = Hand_eye_pose_params_Trans;
    %disp(T_HE);
    T_final = T_robot * T_HE;
    pos = T_final(1:3, 4)';
    %pos = T_robot(1:3, 4)';
    %disp(pos)

    % DH 방식
    %thetah = Hand_eye_pose_params_Rotat(1);
    %alphah = Hand_eye_pose_params_Rotat(2);
    %dh = Hand_eye_pose_params_Trans(3);
    %ah = Hand_eye_pose_params_Trans(1);
    %%disp(dh);
    %%disp(ah);
    %HA = [cosd(thetah), -sind(thetah)*cosd(alphah),  sind(theta)*sind(alphah), ah*cosd(thetah);
    %         sind(thetah),  cosd(thetah)*cosd(alphah), -cosd(theta)*sind(alphah), ah*sind(thetah);
    %         0,           sind(alphah),             cosd(alphah),            dh;
    %         0,           0,                      0,                     1];
    %T_final = T_robot * HA;
    %pos = T_final(1:3, 4)';
    %%pos = T_robot(1:3, 4)';
    %%disp(pos)

end

function R = eul2rotm(eul)
    roll = eul(1);
    pitch = eul(2);
    yaw = eul(3);

    Rx = [1 0 0;
          0 cos(roll) -sin(roll);
          0 sin(roll) cos(roll)];

    Ry = [cos(pitch) 0 sin(pitch);
          0 1 0;
          -sin(pitch) 0 cos(pitch)];

    Rz = [cos(yaw) -sin(yaw) 0;
          sin(yaw) cos(yaw) 0;
          0 0 1];

    R = Rz * Ry * Rx; % ZYX 순서 회전 (추후에 확인)
end

