function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
% 제어기 상태 변수(적분기) 초기화
if isempty(fieldnames(ctrlState)) || ~isfield(ctrlState, 'intError')
    ctrlState.intError = 0;
end

% 1. 능동 전륜 조향(AFS) 제어 로직 (PI 제어 적용)
error_yaw = yawRateRef - yawRate;
% 적분 항 및 Anti-windup(적분 포화 방지) 적용
ctrlState.intError = ctrlState.intError + error_yaw * dt;
ctrlState.intError = max(min(ctrlState.intError, CTRL.LAT.intMax), -CTRL.LAT.intMax);

% 속도 스케줄링 (고속 주행 시 조향 이득을 조정하여 과도한 조향 방지)
v_ref = 20; % 기준 속도 약 72km/h
gain_sched = min(vx / v_ref, 2);

% PI 제어기 출력 계산
deltaAdd.steerAngle = (CTRL.LAT.Kp * error_yaw + CTRL.LAT.Ki * ctrlState.intError) * gain_sched;
deltaAdd.steerAngle = max(min(deltaAdd.steerAngle, LIM.MAX_STEER_ANGLE), -LIM.MAX_STEER_ANGLE);

% 2. 차체 자세 제어(ESC) 제어 로직
beta_th = deg2rad(2.5); % 사이드 슬립 허용 임계값
K_beta = 50000; % 복원 요 모멘트 이득
if abs(slipAngle) > beta_th
    % 슬립 각도에 비례한 복원 요 모멘트 발생
    deltaAdd.yawMoment = -K_beta * sign(slipAngle) * (abs(slipAngle) - beta_th) * gain_sched;
else
    deltaAdd.yawMoment = 0;
end
end