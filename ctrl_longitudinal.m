function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
% 상태 변수(이전 힘, ABS 상태, 적분 오차) 초기화
if isempty(fieldnames(ctrlState)) || ~isfield(ctrlState,'prevForce')
    ctrlState.prevForce = 0;
    ctrlState.abs_active = false;
    ctrlState.intError = 0;
end

% 속도 추종을 위한 PI 제어 수행
error_vx = vxRef - vx;
ctrlState.intError = ctrlState.intError + error_vx*dt;
ctrlState.intError = max(min(ctrlState.intError, CTRL.LON.intMax), -CTRL.LON.intMax);

Kp = 4000;
force_req = Kp*error_vx + CTRL.LON.Ki*ctrlState.intError;

% ABS 영역 판별 및 제어
if error_vx < -1 % 감속 요구 시
    if vx < 2
        ctrlState.abs_active = false;
    else
        % 가속도 기반 ABS 활성화 조건 확인
        if ax > -8.5
            ctrlState.abs_active = true;
        elseif ax < -9.0
            ctrlState.abs_active = false;
        end
    end

    if ctrlState.abs_active
        force_req = force_req*0.7; % 제동력 감쇠를 통한 휠 슬립 제어
    else
        force_req = -15000; % 최대 제동력 인가
    end
end

% Jerk 제한 (제동력 급변 방지)
max_change = LIM.MAX_JERK*1800*dt;
force_req = max(min(force_req, ctrlState.prevForce + max_change), ctrlState.prevForce - max_change);

% 출력 명령 할당
ctrlState.prevForce = force_req;
forceCmd.Fx_total = force_req;
if force_req < 0
    forceCmd.brakeRatio = min(abs(force_req)/15000,1);
else
    forceCmd.brakeRatio = 0;
end
end