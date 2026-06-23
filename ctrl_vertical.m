function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
% 4륜 개별 댐퍼 명령 벡터(감쇠력 계수) 초기화
dampingCmd = zeros(4, 1);

% 1. Skyhook 제어 알고리즘 (각 휠별 수행)
for i = 1:4
    zs_dot = suspState.zs_dot(i); % 차체 절대 속도
    zu_dot = suspState.zu_dot(i); % 휠(현가하질량) 절대 속도

    % 차체 운동 방향과 댐퍼 작동 방향 확인
    % 차체 거동을 억제해야 하는 구간인지 판단
    if zs_dot * (zs_dot - zu_dot) > 0
        % 차체 제어 구간: 감쇠력을 크게 설정 (Hard)
        c_target = CTRL.VER.cMax;
    else
        % 노면 충격 흡수 구간: 감쇠력을 작게 설정 (Soft)
        c_target = CTRL.VER.cMin;
    end

    % 최종 감쇠력 클리핑 (정해진 범위 내 유지)
    dampingCmd(i) = max(min(c_target, CTRL.VER.cMax), CTRL.VER.cMin);
end
end