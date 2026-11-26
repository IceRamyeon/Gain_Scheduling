% evaluate_gs_altstep.m
% (B안) 원 운동 중 상승(+0.3m) 후 원래 고도 복귀(-0.3m) {'up', 'down'} 3-Way 공정 비교
%   1) FF only (Z_Gain_Scheduling = false, PPO OFF)
%   2) GS      (Z_Gain_Scheduling = true,  PPO OFF)
%   3) GS+PPO  (Z_Gain_Scheduling = true,  PPO ON)
% 평가 조건:
%   v = 5.0 m/s, kappa = 0.15 (R=6.67m, phi_eq=20.9 deg) & kappa = 0.20 (R=5.00m, phi_eq=27.0 deg)
%   n_laps = 3, height_step = 0.3 m, height_order = {'up', 'down'} (6.0 m -> 6.3 m -> 6.0 m)

clear; clc; close all;
[px4_dir, ~, ~] = fileparts(mfilename('fullpath'));
parent_dir = fileparts(px4_dir);
cd(px4_dir);
addpath(px4_dir);
addpath(parent_dir);

agent_file = fullfile(px4_dir, 'saved_gs_altstep_ppo_agent.mat');
S = load(agent_file);
if isfield(S, 'agent'), ag_ppo = S.agent; else, ag_ppo = S.saved_agent; end
ag_ppo.UseExplorationPolicy = false;

kappas = [0.15, 0.20];
logs_all = cell(numel(kappas), 3);

for ki = 1:numel(kappas)
    kap = kappas(ki);
    phi_eq_deg = rad2deg(atan((5.0^2 * kap) / 9.81));
    fprintf('\n=========================================================================================\n');
    fprintf('  Circle Up (+0.3m) & Down (-0.3m) Return Test | v=5.0 m/s, kappa=%.2f (phi_eq=%.1f deg)\n', kap, phi_eq_deg);
    fprintf('=========================================================================================\n');

    modes = {'FF only', 'GS (Z-LPV)', 'GS + PPO'};

    for mi = 1:3
        env = RL_DroneEnv_GS_AltStep(false);
        env.randomize_mission    = false;
        env.randomize_delay      = false;
        env.randomize_thrust_exp = false;
        env.randomize_att_loop   = false;
        env.cfg.target_speed     = 5.0;
        env.cfg.curvature        = kap;
        env.cfg.n_laps           = 3;
        env.cfg.height_step      = 0.3;
        env.cfg.height_order     = {'up', 'down'}; % 6.0 -> 6.3 -> 6.0 m
        env.cfg.Z_Gain_Scheduling = (mi >= 2);
        obs = reset(env);

        N_max = env.max_steps;
        t_hist   = zeros(1, N_max);
        h_hist   = zeros(1, N_max);
        hd_hist  = zeros(1, N_max);
        exy_hist = zeros(1, N_max);
        sat_hist = zeros(1, N_max);
        u1_hist  = zeros(1, N_max);

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
            u1_hist(step_k)  = env.prev_U1_des;
        end

        t_hist   = t_hist(1:step_k);
        h_hist   = h_hist(1:step_k);
        hd_hist  = hd_hist(1:step_k);
        exy_hist = exy_hist(1:step_k);
        sat_hist = sat_hist(1:step_k);
        u1_hist  = u1_hist(1:step_k);

        t_s1 = env.traj.t_steps(1); % Up step: 6.0 -> 6.3 m
        t_s2 = env.traj.t_steps(2); % Down step: 6.3 -> 6.0 m
        t_c1 = env.traj.t_circ1;
        dh   = 0.3;
        band = 0.10 * dh; % ±3 cm

        % [1] 상승 계단 (t_s1 ~ t_s2): 6.0 -> 6.3 m
        idx1 = find(t_hist >= t_s1 & t_hist < t_s2);
        ts1  = t_hist(idx1) - t_s1; hs1 = h_hist(idx1);
        i10_u = find(hs1 >= 6.0 + 0.1*dh, 1, 'first');
        i90_u = find(hs1 >= 6.0 + 0.9*dh, 1, 'first');
        if ~isempty(i10_u) && ~isempty(i90_u), tr_up = ts1(i90_u) - ts1(i10_u); else, tr_up = NaN; end
        os_up = max(0, (max(hs1) - 6.3) / dh * 100);
        oi_u  = find(abs(hs1 - 6.3) > band, 1, 'last');
        if isempty(oi_u), ts_up = 0; elseif oi_u < numel(ts1), ts_up = ts1(oi_u + 1); else, ts_up = NaN; end
        ess_up = abs(6.3 - mean(hs1(max(1, end-30):end))) * 1000; % [mm]

        % [2] 하강 복귀 계단 (t_s2 ~ t_c1): 6.3 -> 6.0 m
        idx2 = find(t_hist >= t_s2 & t_hist <= t_c1);
        ts2  = t_hist(idx2) - t_s2; hs2 = h_hist(idx2);
        i10_d = find(hs2 <= 6.3 - 0.1*dh, 1, 'first');
        i90_d = find(hs2 <= 6.3 - 0.9*dh, 1, 'first');
        if ~isempty(i10_d) && ~isempty(i90_d), tr_dn = ts2(i90_d) - ts2(i10_d); else, tr_dn = NaN; end
        os_dn = max(0, (6.0 - min(hs2)) / dh * 100);
        oi_d  = find(abs(hs2 - 6.0) > band, 1, 'last');
        if isempty(oi_d), ts_dn = 0; elseif oi_d < numel(ts2), ts_dn = ts2(oi_d + 1); else, ts_dn = NaN; end
        ess_dn = abs(6.0 - mean(hs2(max(1, end-30):end))) * 1000; % [mm]

        % [3] 계단 구간(t_s1 ~ t_c1) Z RMSE 및 원 전체 구간 XY RMSE
        idx_step_all = find(t_hist >= t_s1 & t_hist <= t_c1);
        idx_c        = find(t_hist >= env.traj.t_circ0 & t_hist <= t_c1);
        z_rmse_step  = sqrt(mean((h_hist(idx_step_all) - hd_hist(idx_step_all)).^2));
        z_rmse_all   = sqrt(mean((h_hist(idx_c) - hd_hist(idx_c)).^2));
        xy_rmse      = sqrt(mean(exy_hist(idx_c).^2));

        fprintf('%-10s | [UP]   Rise=%.2fs OS=%4.1f%% Settle=%5.2fs |E_ss|=%4.1fmm\n', ...
            modes{mi}, tr_up, os_up, ts_up, ess_up);
        fprintf('           | [DOWN] Rise=%.2fs OS=%4.1f%% Settle=%5.2fs |E_ss|=%4.1fmm | Z_RMSE(step/all)=%.4f/%.4fm | XY_RMSE=%.4fm\n', ...
            tr_dn, os_dn, ts_dn, ess_dn, z_rmse_step, z_rmse_all, xy_rmse);

        logs_all{ki, mi} = struct('t', t_hist, 'h', h_hist, 'hd', hd_hist, ...
            't_s1', t_s1, 't_s2', t_s2, 't_c1', t_c1, 'u1', u1_hist, 'exy', exy_hist);
    end
end

h_fig = figure('Color', [1 1 1], 'Position', [100, 100, 1450, 780], 'Visible', 'off');
for ki = 1:numel(kappas)
    kap = kappas(ki);
    L1 = logs_all{ki, 1}; L2 = logs_all{ki, 2}; L3 = logs_all{ki, 3};
    t0 = L1.t_s1;

    subplot(2, 2, (ki-1)*2 + 1);
    plot(L1.t - t0, L1.hd, 'k--', 'LineWidth', 1.6, 'DisplayName', 'Target (6.0 -> 6.3 -> 6.0m)'); hold on;
    plot(L1.t - t0, L1.h,  'r-.', 'LineWidth', 1.4, 'DisplayName', '1. FF only');
    plot(L2.t - t0, L2.h,  'g-',  'LineWidth', 1.5, 'DisplayName', '2. GS (Z-LPV)');
    plot(L3.t - t0, L3.h,  'b-',  'LineWidth', 1.8, 'DisplayName', '3. GS + PPO');
    yline(6.3 + 0.03, ':', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
    yline(6.3 - 0.03, ':', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
    yline(6.0 + 0.03, ':', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
    yline(6.0 - 0.03, ':', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
    xlim([-3, (L1.t_c1 - t0) + 1]); ylim([5.88, 6.42]);
    set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on; box on;
    title(sprintf('Up (+0.3m) & Down (-0.3m) Altitude Response (kappa=%.2f, v=5 m/s)', kap), 'Color', [0 0 0], 'FontWeight', 'bold');
    xlabel('Time since Up-Step [s]', 'Color', [0 0 0]); ylabel('Altitude [m]', 'Color', [0 0 0]);
    legend('Location', 'northeast', 'TextColor', [0 0 0], 'Color', [1 1 1]);

    subplot(2, 2, (ki-1)*2 + 2);
    plot(L1.t - t0, L1.exy, 'r-.', 'LineWidth', 1.4, 'DisplayName', '1. FF only'); hold on;
    plot(L2.t - t0, L2.exy, 'g-',  'LineWidth', 1.5, 'DisplayName', '2. GS (Z-LPV)');
    plot(L3.t - t0, L3.exy, 'b-',  'LineWidth', 1.8, 'DisplayName', '3. GS + PPO');
    xlim([-3, (L1.t_c1 - t0) + 1]);
    set(gca, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0]); grid on; box on;
    title(sprintf('Horizontal (XY) Tracking Error (kappa=%.2f, v=5 m/s)', kap), 'Color', [0 0 0], 'FontWeight', 'bold');
    xlabel('Time since Up-Step [s]', 'Color', [0 0 0]); ylabel('XY Error [m]', 'Color', [0 0 0]);
    legend('Location', 'northeast', 'TextColor', [0 0 0], 'Color', [1 1 1]);
end
saveas(h_fig, fullfile(px4_dir, 'GS_AltStep_Comparison.png'));
saveas(h_fig, fullfile(px4_dir, 'GS_AltStep_UpDown_Comparison.png'));
close(h_fig);
fprintf('\n>>> Saved plot: GS_AltStep_UpDown_Comparison.png\n');

% 에피소드별(500 단위: 0 ~ 6000) 학습 성능 향상 추이 평가 (kappa=0.15 및 kappa=0.20, Up+Down 코스)
ckpts = 0:500:6000;
ckpt_dir = fullfile(px4_dir, 'savedAgents_gs_altstep');
for kap_ev = [0.15, 0.20]
    fprintf('\n=========================================================================================\n');
    fprintf('  Learning Progression (Ep 0 -> 500 -> ... -> 6000) | kappa=%.2f, v=5.0m/s, Up+Down 0.3m\n', kap_ev);
    fprintf('=========================================================================================\n');
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
        env.cfg.target_speed = 5.0; env.cfg.curvature = kap_ev; env.cfg.n_laps = 3;
        env.cfg.height_step = 0.3; env.cfg.height_order = {'up', 'down'};
        env.cfg.Z_Gain_Scheduling = true;
        obs = reset(env);

        N_max = env.max_steps;
        t_h = zeros(1, N_max); h_h = zeros(1, N_max); hd_h = zeros(1, N_max); exy_h = zeros(1, N_max);
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
            exy_h(k_s) = norm(p_d(1:2) - st(1:2));
        end
        t_h = t_h(1:k_s); h_h = h_h(1:k_s); hd_h = hd_h(1:k_s); exy_h = exy_h(1:k_s);
        t_s1 = env.traj.t_steps(1); t_s2 = env.traj.t_steps(2); t_c1 = env.traj.t_circ1;
        idx1 = find(t_h >= t_s1 & t_h < t_s2);
        idx2 = find(t_h >= t_s2 & t_h <= t_c1);
        os_u = max(0, (max(h_h(idx1)) - 6.3) / 0.3 * 100);
        os_d = max(0, (6.0 - min(h_h(idx2))) / 0.3 * 100);
        idx_st = find(t_h >= t_s1 & t_h <= t_c1);
        idx_c  = find(t_h >= env.traj.t_circ0 & t_h <= t_c1);
        fprintf('Ep %4d | OS(Up/Dn)=%4.1f%%/%4.1f%% | Z_RMSE(step/all)=%.4f/%.4fm | XY_RMSE=%.4fm\n', ...
            ep_num, os_u, os_d, sqrt(mean((h_h(idx_st)-hd_h(idx_st)).^2)), ...
            sqrt(mean((h_h(idx_c)-hd_h(idx_c)).^2)), sqrt(mean(exy_h(idx_c).^2)));
    end
end
