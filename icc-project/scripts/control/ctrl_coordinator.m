function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
% 휠 유효 반경 설정 (차량 정보에 정의된 경우 사용)
r_w = 0.33;
if isfield(VEH, 'r_w')
    r_w = VEH.r_w;
end

% 1. 종방향 하중 이동을 고려한 기본 제동력 분배 (Front 60% : Rear 40% 비율)
if lonCmd.Fx_total < 0
    F_brake_total = abs(lonCmd.Fx_total);
    F_f_total = F_brake_total * 0.60; % 전륜 제동력 할당
    F_r_total = F_brake_total * 0.40; % 후륜 제동력 할당

    % 휠 토크로 변환 (T = F * r)
    T_f_base = (F_f_total / 2) * r_w;
    T_r_base = (F_r_total / 2) * r_w;
else
    T_f_base = 0;
    T_r_base = 0;
end
% 기본 제동 토크 벡터 [FL; FR; RL; RR]
base_brake = [T_f_base; T_f_base; T_r_base; T_r_base]; 

% 2. 차체 자세 제어(ESC)를 위한 횡방향 요 모멘트(Yaw Moment) 분배
Mz = latCmd.yawMoment;
ratio_f = 0.6; % 전륜 모멘트 할당 비율

% 좌우 휠의 차동 제동을 통한 요 모멘트 생성
dT_f = (Mz * ratio_f) / VEH.track_f * r_w;
dT_r = (Mz * (1 - ratio_f)) / VEH.track_r * r_w;
brake_diff = [dT_f; -dT_f; dT_r; -dT_r];

% 3. 종/횡방향 요구 토크 통합 및 최종 제어 명령 할당
total_brake = base_brake + brake_diff;
% 제동 토크 한계(LIM.MAX_BRAKE_TRQ) 적용 및 0 이상으로 제한
actuatorCmd.brakeTorque = max(min(total_brake, LIM.MAX_BRAKE_TRQ), 0);

% 조향각 및 수직 댐핑 계수 한계치 적용
actuatorCmd.steerAngle = max(min(latCmd.steerAngle, LIM.MAX_STEER_ANGLE), -LIM.MAX_STEER_ANGLE);
actuatorCmd.dampingCoeff = verCmd;
end