function Verification()
    % csv에서 데이터 불러오기
    data = readmatrix('Verification_20_set.xlsx', 'Range', [4, 2, 23, 9]); % 시작 셀 위치 ~ 끝 셀 위치
    %disp(data)
    data_c = readmatrix('Verification_20_set.xlsx', 'Range', [3, 10, 9, 15]); % 시작 셀 위치 ~ 끝 셀 위치
    disp(data_c)

    % Calibration Sphere의 실제 좌표 (광학테이블)
    real_sphere_coordinates = [500, 10, 75]; %Unity상 로봇 원점 Y+ 10mm 고려(광학테이블)

    Number_of_position = 1;
    Number_of_measurement = 20;

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



    % Original Parameters
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
            DH_params(i, j) = {DH_i_j};
            %disp(DH_params{i,j})
        end
    end
    Hand_eye_pose = [0, -32, 74.5, 0.0, 0.0, 0.0];

    % Calibrated Parameters
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
            Calibrated_DH_i_j = [data_c(1,1), data_c(1,2), data_c(1,3), data_c(1,4);
                                data_c(2,1), data_c(2,2), data_c(2,3), data_c(2,4);
                                data_c(3,1), data_c(3,2), data_c(3,3), data_c(3,4);
                                data_c(4,1), data_c(4,2), data_c(4,3), data_c(4,4);
                                data_c(5,1), data_c(5,2), data_c(5,3), data_c(5,4);
                                data_c(6,1), data_c(6,2), data_c(6,3), data_c(6,4);
                                ];
            %disp(Calibrated_DH_i_j)
            DH_i_j = DH_i_j + Calibrated_DH_i_j;
            Calibrated_DH_params(i, j) = {DH_i_j};
            %disp(initial_DH_params{i,j})
        end
    end
    Calibrated_Hand_eye_pose = [0, -32, 74.5, 0.0, 0.0, 0.0];
    Calibrated_Handeye_Params = [data_c(7,1), data_c(7,2), data_c(7,3), data_c(7,4), data_c(7,5), data_c(7,6)];
    %disp(Calibrated_Handeye_Params)
    Calibrated_Hand_eye_pose = Calibrated_Hand_eye_pose + Calibrated_Handeye_Params;

    Predicted_Original_M = zeros(1, 20);
    Predicted_Calibrated_M = zeros(1, 20);
    for i = 1:1
        for j = 1:20
            Original_POS = forward_kinematics(DH_params{i, j}, Hand_eye_pose);
            %disp(Original_POS);
            Predicted_Original = norm(Original_POS - real_sphere_coordinates(i, :));
            %disp(Predicted_Original)
            Predicted_Original_M(i, j) = Predicted_Original;            
        end
    end

    for i = 1:1
        for j = 1:20
            Calibrated_POS = forward_kinematics(Calibrated_DH_params{i, j}, Calibrated_Hand_eye_pose);
            %disp(Calibrated_POS);
            Predicted_Calibrated = norm(Calibrated_POS - real_sphere_coordinates(i, :));
            Predicted_Calibrated_M(i, j) = Predicted_Calibrated;       
        end
    end

    %disp(Measured_distance_camera_sphere);
    %disp(real_sphere_coordinates(i, :))
    Original_errors = Measured_distance_camera_sphere - Predicted_Original_M;
    disp("Errors Using Original Parameters: ")
    disp(Original_errors(:))

    Calibrated_errors = Measured_distance_camera_sphere - Predicted_Calibrated_M;
    disp("Errors Using Calibrated Parameters: ")
    disp(Calibrated_errors(:))

end


function pos = forward_kinematics(DH_params, Hand_eye_params)
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
    Hand_eye_pose_params_Trans = Hand_eye_params(1:3);
    Hand_eye_pose_params_Rotat = Hand_eye_params(4:6);
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
    %disp(pos)
end
