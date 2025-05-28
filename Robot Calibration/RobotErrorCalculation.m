function Levenberg_Marquardt()

    % % 초기 추정값 (협업 로봇의 D-H 파라미터 입력)
    % % DH 값 .csv에서 행렬 추출
    % DH_s_n = {DH_j1, DH_j2, DH_j3, DH_j4, DH_j5, DH_j6}; % s = 구 위치 변화 n = 측정 건수
    % DH_i_n = [a_n, d_n, alpha_n, theta_n];
    % for i = 1:s
    %     for n = 1:n
    %         initial_DH_params(i, j) = {DH_s_n};
    %     end
    % end
    DH_params_matrix_size = rand(1, 24);
    for row = 1:5
        for column = 1:6
            initial_DH_params(row, column) = {rand(1, 24)};
        end
    end
    % disp(initial_DH_params{1,1});

    % % 초기 추정값 (Gocator Pose 값 입력) 해당 값은 Line-Laser에서 구의 R를 통해 간접적으로 추론할 것
    % H_E_POS = [X, Y, Z];
    % H_E_ORI = [Rx, Ry, Rz];
    initial_hand_eye_pose = [0.5, 0.5, 0.5, 0.1, 0.2, 0];

    % % Measured Values
    % R = 0.05 %(Calibration sphere의 반지름)
    Measured_distance_camera_sphere = [ 0.1, 0.5, 0.6, 0.3, 0.9, 1.3;
                                        0.5, 0.8, 1.5, 1.8, 1.6, 1.6;
                                        0.2, 0.5, 0.8, 1.3, 1.8, 2.6;
                                        0.3, 0.6, 1.0, 1.5, 1.1, 2.6;
                                        0.3, 0.7, 0.5, 0.2, 1.8, 0.9;
                                        ];

    % Calibration Sphere의 실제 좌표 (광학테이블)
    real_sphere_coordinates = [ 0.4, 0.5, 0.2;
                                0.4, 0.8, 0.3;
                                0.2, 0.8, 0.4;
                                0.2, 0.5, 0.4;
                                0.3, 0.5, 0.3;
                                ];

    % 옵션 Settings: Levenberg-Marquardt
    initial_deltas = zeros(1, length(DH_params_matrix_size) + length(initial_hand_eye_pose));
    options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt', ...
        'Display', 'iter', 'MaxIterations', 10000);

    % lsqnonlin 최적화 수행
    optimized_deltas = lsqnonlin(@(params) error_function(params, Measured_distance_camera_sphere, ...
        initial_DH_params, initial_hand_eye_pose, real_sphere_coordinates), initial_deltas, [], [], options);

    disp('Optimized Parameters:');
    disp(optimized_deltas);

end

% 오차 추정
function errors = error_function(params, Measured_distance_camera_sphere, initial_DH_params, initial_hand_eye_pose, real_sphere_coordinates)

    % D-H params
    delta_DH_params = params(1:24);
    % disp(delta_DH_params);

    % Hand-eye params
    delta_Hand_eye_pose_params_Trans = params(25:27);
    delta_Hand_eye_pose_params_Rotat = params(28:30);
    Hand_eye_pose_params_Trans = initial_hand_eye_pose(1:3) + delta_Hand_eye_pose_params_Trans;
    Hand_eye_pose_params_Rotat = initial_hand_eye_pose(4:6) + delta_Hand_eye_pose_params_Rotat;

    num_points_1 = size(Measured_distance_camera_sphere, 1); % 위치 개수
    num_points_2 = size(Measured_distance_camera_sphere, 2); % 위치별 측정 건수
    Predicted_distance = zeros(num_points_1, num_points_2);

    for i = 1:num_points_1
        for j = 1:num_points_2
            Robot_Position_Calculation = forward_kinematics(initial_DH_params{i, j} + delta_DH_params);
            Camera_Position_Calculation = Hand_eye_transformation(Robot_Position_Calculation, ...
                Hand_eye_pose_params_Trans, Hand_eye_pose_params_Rotat);
            % disp(Camera_Position_Calculation);
            Predicted_length_of_camera_sphere = norm(Camera_Position_Calculation - real_sphere_coordinates(i, :));
            Predicted_distance(i, j) = Predicted_length_of_camera_sphere;
        end
    end

    disp(Predicted_distance);

    % 오차 계산
    errors = Measured_distance_camera_sphere - Predicted_distance;
    errors = errors(:);
    % disp('Error values:');
    % disp(errors);

end

% Forward kinematics
function pos = forward_kinematics(DH_params)
    T = eye(4);

    for i = 1:6
        theta = DH_params(4*i - 3);  % 조인트 각도
        d     = DH_params(4*i - 2);  % 링크 오프셋
        a     = DH_params(4*i - 1);  % 링크 길이
        alpha = DH_params(4*i);      % 링크 회전 각도

        % Matrix Transformation
        A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
             sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
             0,           sin(alpha),             cos(alpha),            d;
             0,           0,                      0,                     1];

        T = T * A;
    end

    pos = T(1:3, 4)';
end

% Hand-eye Transformation Matrix
function Hand_eye_pose = Hand_eye_transformation(Position_Calculation, ...
    Hand_eye_pose_params_Trans, Hand_eye_pose_params_Rotat)

    % 회전 행렬 추출
    R = eul2rotm(Hand_eye_pose_params_Rotat);

    % 회전 적용 + 이동
    Hand_eye_pose = (R * Position_Calculation')' + Hand_eye_pose_params_Trans;

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

