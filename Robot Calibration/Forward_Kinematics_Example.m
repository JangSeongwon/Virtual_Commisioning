function Forward_Kinematics_Example()
    % csv에서 데이터 불러오기
    data = readmatrix('RobotJointExample.xlsx', 'Range', [1, 1, 31, 6]); % 시작 셀 위치 ~ 끝 셀 위치
    %disp(data)
    Number_of_position = 1;
    Number_of_measurement = 30;
    %Number_of_measurement = size(data, 1);

    for i = 1:Number_of_position
        for  j = 1:Number_of_measurement
            % theta, d, a, alpha
            DH_i_j = [-11.8557, 152.5, 0, -90;
                      5.7779-90, 200, 411, 0; %두번째 조인트가 양수일때 X+ 방향
                      127.6595-90, -165.5, 0, -90;
                      62.4717, 368, 0, 90;
                      20.3768, 0, 0, -90;
                      -70.1888, 121, 0, 0;
                     ];
            DH_params_ori(i, j) = {DH_i_j};
            %disp(DH_params_ori{i,j})
        end
    end

    for i = 1:1
        for j = 1:Number_of_measurement
            T_robot = eye(4);
            DH_params = DH_params_ori{i,j};
            for m = 1:6
                theta = DH_params(m, 1);  % 조인트 각도
                d = DH_params(m, 2);  % 링크 오프셋
                a = DH_params(m, 3);  % 링크 길이
                alpha = DH_params(m, 4);  % 링크 회전 각도
                %disp(theta)
                %disp(d)
                %disp(a)
                %disp(alpha)
                theta = mod(theta + 180, 360) - 180;
        
                % Matrix Transformation
                A = [cosd(theta), -sind(theta)*cosd(alpha),  sind(theta)*sind(alpha), a*cosd(theta);
                     sind(theta),  cosd(theta)*cosd(alpha), -cosd(theta)*sind(alpha), a*sind(theta);
                     0,           sind(alpha),             cosd(alpha),            d;
                     0,           0,                      0,                     1];
                %MA = [cosd(theta), -sind(theta), 0, a;
                %    sind(theta)*cosd(alpha), cosd(theta)*cosd(alpha), -sind(alpha), -d*sind(alpha);
                %    sind(theta)*sind(alpha), cosd(theta)*sind(alpha), cosd(alpha), d*cosd(alpha);
                %    0,0,0,1];
        
                %disp(A)
                T_robot = T_robot * A;
                %disp(T_robot)
            end
             pos = T_robot(1:3, 4)';
             disp(pos)

        end
    end
end

