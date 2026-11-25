% evaluate_gs_altstep.m
% (B안) 원 운동 중 +0.3m 고도 계단 추종 3-Way 공정 비교 (동일 기반 제어기 PositionCtrl_GS)
%   1) FF only (Z_Gain_Scheduling = false, PPO OFF)
%   2) GS      (Z_Gain_Scheduling = true,  PPO OFF)
%   3) GS+PPO  (Z_Gain_Scheduling = true,  PPO ON)
% 평가 조건: ChoeJae px4_main.m (f02c317) 동일
%   v = 5.0 m/s, kappa = 0.15 (R=6.67m, phi_eq=21 deg) & kappa = 0.25 (R=4.0m, phi_eq=32.5 deg)
%   n_laps = 3, height_step = 0.3 m, height_order = {'up'} (6.0 m -> 6.3 m)

clear; clc; close all;
[px4_dir, ~, ~] = fileparts(mfilename('fullpath'));
parent_dir = fileparts(px4_dir);
cd(px4_dir);
addpath(px4_dir);
addpath(parent_dir);

agent_file = fullfile(px4_dir, 'saved_gs_altstep_ppo_agent.mat');
has_agent = exist(agent_file, 'file') == 2;
if has_agent
    S = load(agent_file);
    if isfield(S, 'agent'), ag_ppo = S.agent; else, ag_ppo = S.saved_agent; end
    ag_ppo.UseExplorationPolicy = false;
else
    ag_ppo = [];
    fprintf('[INFO] 학습된 에이전트가 아직 없어 FF only vs GS 기준선만 평가합니다.\n');
end

kappas = [0.15, 0.20];
logs_all = cell(numel(kappas), 3);

for ki = 1:numel(kappas)
    kap = kappas(ki);
    phi_eq_deg = rad2deg(atan((5.0^2 * kap) / 9.81));
    fprintf('\n================================================================================\n');
    fprintf('  Circle +0.3m Altitude Step | v=5.0 m/s, kappa=%.2f (R=%.2fm, phi_eq=%.1f deg)\n', kap, 1/kap, phi_eq_deg);
    fprintf('================================================================================\n');

    modes = {'FF only', 'GS (Z-LPV)', 'GS + PPO'};
    n_m = 2 + double(has_agent);

    for mi = 1:n_m
        env = RL_DroneEnv_GS_AltStep(false);
        env.randomize_mission    = false;
        env.randomize_delay      = false;
        env.randomize_thrust_exp = false;
        env.randomize_att_loop   = false;
        env.cfg.target_speed     = 5.0;
        env.cfg.curvature        = kap;
        env.cfg.n_laps           = 3;
        env.cfg.height_step      = 0.3;
        env.cfg.height_order     = {'up'};
        env.cfg.Z_Gain_Scheduling = (mi >= 2);
        obs = reset(env);

        N_max = env.max_steps;
        t_hist   = zeros(1, N_max);
        h_hist   = zeros(1, N_max);
        hd_hist  = zeros(1, N_max);
        exy_hist = zeros(1, N_max);
        sat_hist = zeros(1, N_max);
        pz_hist  = zeros(1, N_max);
        dz_hist  = zeros(1, N_max);

        step_k = 0;
        done = false;
        while ~done && step_k < N_max
            step_k = step_k + 1;
            if mi == 3
                a_out = getAction(ag_ppo, obs);
                if iscell(a_out), act = a_out{1}; else, act = a_out; end
                act = max(-1, min(1, double(act(:))));
            else
                act = zeros(4, 1);
            end
            [obs, ~, done] = step(env, act);
            st = env.drone_px4.GetState();
            [p_d, ~, ~, ~] = env.traj.get_position(env.drone_px4.t);
            t_hist(step_k)   = env.drone_px4.t;
            h_hist(step_k)   = -st(3);
            hd_hist(step_k)  = -p_d(3);
            exy_hist(step_k) = norm(p_d(1:2) - st(1:2));
            sat_hist(step_k) = double(env.drone_px4.is_saturated);
            gz = env.posCtrl.GetZ_PID();
            pz_hist(step_k)  = gz(1);
            dz_hist(step_k)  = gz(3);
        end

        t_hist   = t_hist(1:step_k);
        h_hist   = h_hist(1:step_k);
        hd_hist  = hd_hist(1:step_k);
        exy_hist = exy_hist(1:step_k);
        sat_hist = sat_hist(1:step_k);
        pz_hist  = pz_hist(1:step_k);
        dz_hist  = dz_hist(1:step_k);

        % 계단 응답 지표 계산 (t_step ~ t_circ1 원 선회 구간 내)
        t_step = env.traj.t_steps(1);
        idx_s  = find(t_hist >= t_step & t_hist <= env.traj.t_circ1);
        ts     = t_hist(idx_s) - t_step;
        hs     = h_hist(idx_s);
        h0     = 6.0;
        dh     = 0.3;
        h_tgt  = h0 + dh;
        band   = 0.10 * dh; % ±3 cm

        % 상승시간 (10% -> 90%)
        i10 = find(hs >= h0 + 0.1*dh, 1, 'first');
        i90 = find(hs >= h0 + 0.9*dh, 1, 'first');
        if ~isempty(i10) && ~isempty(i90), t_rise = ts(i90) - ts(i10); else, t_rise = NaN; end

        % 오버슈트 (%)
        overshoot_pct = max(0, (max(hs) - h_tgt) / dh * 100);

        % ±10% 정착시간
        out_idx = find(abs(hs - h_tgt) > band, 1, 'last');
        if isempty(out_idx), t_settle = 0; elseif out_idx < numel(ts), t_settle = ts(out_idx + 1); else, t_settle = NaN; end

        % 원 구간 전체 Z RMSE & XY RMSE
        idx_c = find(t_hist >= env.traj.t_circ0 & t_hist <= env.traj.t_circ1);
        z_rmse  = sqrt(mean((h_hist(idx_c) - hd_hist(idx_c)).^2));
        xy_rmse = sqrt(mean(exy_hist(idx_c).^2));
        sat_pct = mean(sat_hist(idx_c)) * 100;

        fprintf('%-10s | Rise=%.2fs | OS=%5.1f%% | Settle(10%%)=%.2fs | Z_RMSE=%.4fm | XY_RMSE=%.4fm | Sat=%.1f%%\n', ...
            modes{mi}, t_rise, overshoot_pct, t_settle, z_rmse, xy_rmse, sat_pct);

        logs_all{ki, mi} = struct('t', t_hist, 'h', h_hist, 'hd', hd_hist, ...
            't_step', t_step, 'pz', pz_hist, 'dz', dz_hist, 'exy', exy_hist);
    end
end

if has_agent
    h_fig = figure('Color', [1 1 1], 'Position', [100, 100, 1400, 750], 'Visible', 'off');
    for ki = 1:numel(kappas)
        kap = kappas(ki);
        L1 = logs_all{ki, 1}; L2 = logs_all{ki, 2}; L3 = logs_all{ki, 3};
        t0 = L1.t_step;

        subplot(2, 2, (ki-1)*2 + 1);
        plot(L1.t - t0, L1.hd, 'k--', 'LineWidth', 1.6, 'DisplayName', 'Target (+0.3m)'); hold on;
        plot(L1.t - t0, L1.h,  'r-.', 'LineWidth', 1.4, 'DisplayName', '1. FF only');
        plot(L2.t - t0, L2.h,  'g-',  'LineWidth', 1.5, 'DisplayName', '2. GS (Z-LPV)');
        plot(L3.t - t0, L3.h,  'b-',  'LineWidth', 1.8, 'DisplayName', '3. GS + PPO');
        yline(6.3 + 0.03, ':', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
        yline(6.3 - 0.03, ':', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
        xlim([-2, 10]); ylim([5.92, 6.42]);
        set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on; box on;
        title(sprintf('Altitude Step Response (kappa=%.2f, v=5 m/s)', kap), 'Color', [0 0 0], 'FontWeight', 'bold');
        xlabel('Time since Step [s]', 'Color', [0 0 0]); ylabel('Altitude [m]', 'Color', [0 0 0]);
        legend('Location', 'southeast', 'TextColor', [0 0 0], 'Color', [1 1 1]);

        subplot(2, 2, (ki-1)*2 + 2);
        plot(L2.t - t0, L2.pz, 'g--', 'LineWidth', 1.4, 'DisplayName', 'GS Base K_{P,z}'); hold on;
        plot(L3.t - t0, L3.pz, 'b-',  'LineWidth', 1.7, 'DisplayName', 'GS+PPO K_{P,z}');
        plot(L2.t - t0, L2.dz, 'm--', 'LineWidth', 1.4, 'DisplayName', 'GS Base K_{D,z}');
        plot(L3.t - t0, L3.dz, 'c-',  'LineWidth', 1.7, 'DisplayName', 'GS+PPO K_{D,z}');
        xlim([-2, 10]);
        set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on; box on;
        title(sprintf('Effective Z PID Gains (kappa=%.2f)', kap), 'Color', [0 0 0], 'FontWeight', 'bold');
        xlabel('Time since Step [s]', 'Color', [0 0 0]); ylabel('Gain [N/m or N/(m/s)]', 'Color', [0 0 0]);
        legend('Location', 'best', 'TextColor', [0 0 0], 'Color', [1 1 1]);
    end
    saveas(h_fig, fullfile(px4_dir, 'GS_AltStep_Comparison.png'));
    close(h_fig);
    fprintf('\n>>> Saved plot: GS_AltStep_Comparison.png\n');

    % 에피소드별(500 단위: 0 ~ 4000) 학습 성능 향상 추이 평가 (kappa=0.15, v=5.0m/s, +0.3m)
    ckpts = 0:500:4000;
    ckpt_dir = fullfile(px4_dir, 'savedAgents_gs_altstep');
    fprintf('\n================================================================================\n');
    fprintf('  Learning Progression (Ep 0 -> 500 -> ... -> 4000) | kappa=0.15, +0.3m\n');
    fprintf('================================================================================\n');
    for ci = 1:numel(ckpts)
        ep_num = ckpts(ci);
        f_ckpt = fullfile(ckpt_dir, sprintf('Agent%d.mat', ep_num));
        if ~exist(f_ckpt, 'file'), continue; end
        Sc = load(f_ckpt);
        if isfield(Sc, 'saved_agent'), ag_c = Sc.saved_agent; else, ag_c = Sc.agent; end
        ag_c.UseExplorationPolicy = false;

        env = RL_DroneEnv_GS_AltStep(false);
        env.randomize_mission = false; env.randomize_delay = false;
        env.randomize_thrust_exp = false; env.randomize_att_loop = false;
        env.cfg.target_speed = 5.0; env.cfg.curvature = 0.15; env.cfg.n_laps = 3;
        env.cfg.height_step = 0.3; env.cfg.height_order = {'up'};
        env.cfg.Z_Gain_Scheduling = true;
        obs = reset(env);

        N_max = env.max_steps;
        t_h = zeros(1, N_max); h_h = zeros(1, N_max); hd_h = zeros(1, N_max);
        exy_h = zeros(1, N_max); sat_h = zeros(1, N_max);
        k_s = 0; done = false;
        while ~done && k_s < N_max
            k_s = k_s + 1;
            a_o = getAction(ag_c, obs);
            if iscell(a_o), act = a_o{1}; else, act = a_o; end
            act = max(-1, min(1, double(act(:))));
            [obs, ~, done] = step(env, act);
            st = env.drone_px4.GetState();
            [p_d, ~, ~, ~] = env.traj.get_position(env.drone_px4.t);
            t_h(k_s) = env.drone_px4.t; h_h(k_s) = -st(3); hd_h(k_s) = -p_d(3);
            exy_h(k_s) = norm(p_d(1:2) - st(1:2)); sat_h(k_s) = double(env.drone_px4.is_saturated);
        end
        t_h = t_h(1:k_s); h_h = h_h(1:k_s); hd_h = hd_h(1:k_s); exy_h = exy_h(1:k_s); sat_h = sat_h(1:k_s);
        t_st = env.traj.t_steps(1);
        idx_s = find(t_h >= t_st & t_h <= env.traj.t_circ1);
        ts = t_h(idx_s) - t_st; hs = h_h(idx_s);
        i10 = find(hs >= 6.03, 1, 'first'); i90 = find(hs >= 6.27, 1, 'first');
        if ~isempty(i10) && ~isempty(i90), tr = ts(i90) - ts(i10); else, tr = NaN; end
        os = max(0, (max(hs) - 6.3) / 0.3 * 100);
        oi = find(abs(hs - 6.3) > 0.03, 1, 'last');
        if isempty(oi), tset = 0; elseif oi < numel(ts), tset = ts(oi + 1); else, tset = NaN; end
        idx_c = find(t_h >= env.traj.t_circ0 & t_h <= env.traj.t_circ1);
        fprintf('Ep %4d    | Rise=%.2fs | OS=%5.1f%% | Settle(10%%)=%.2fs | Z_RMSE=%.4fm | XY_RMSE=%.4fm | Sat=%.1f%%\n', ...
            ep_num, tr, os, tset, sqrt(mean((h_h(idx_c)-hd_h(idx_c)).^2)), sqrt(mean(exy_h(idx_c).^2)), mean(sat_h(idx_c))*100);
    end
end
