% =========================================================================
%            电机无传感器控制 - SMO+PLL 模型初始化脚本
% =========================================================================
clc;
clear;
close all;

fprintf('Initializing model parameters...\n');

% --- 1. 基础电机物理参数 ---
% Rs = 0.14;       % 相电阻 (Ohm)
% Ld = 2.5E-5;     % d轴电感 (H)
% Lq = 2.5E-5;     % q轴电感 (H)
% psi_f = 0.00217; % 永磁体磁链 (Wb)
% Pn = 7;          % 极对数 
% J = 0.0002;      % 转动惯量 (kg*m^2)
% Bs 和 Tm 的单位和定义可能需要根据您的模型确认，这里暂时保留
Bs = 1E-5;       % 粘滞摩擦系数 (N*m*s/rad)
Tm = 0.5;        % 库仑摩擦 (N*m)

% ==========================================
%    选用 2804 云台电机参数 (完美适合调试 SMO)
% ==========================================

% 1. 物理参数
Rs = 5.1;          % 相电阻 5.1 Ohm (云台电机电阻通常很大)
Ld = 2.8e-3;       % d轴电感 2.8 mH (注意是e-3)
Lq = 2.8e-3;       % q轴电感 2.8 mH
psi_f = 0.0078;    % 磁链 (图片中直接给了 0.0078)
Pn = 7;            % 极对数 (12N14P 就是 7对极)
J = 0.0002;        % 转动惯量 (先保持不变，仿真没问题)

% 2. 必须重新计算的部分 (非常重要！！！)
% 因为 L 变大了100倍，你的电流环 PI 参数必须重新算
% 运行你脚本里原本有的那个自动计算公式：
% Kp_current = Ld * bw_current;
% Ki_current = Rs * bw_current;

% 3. SMO 增益也必须更新
% SMO_B_gain = Ts / Ld;  <-- 这个值会变小很多，但这是对的
% SMO_A_gain = 1 - R*Ts/L;

% --- 2. 仿真与控制参数 ---
% 50us = 20kHz (推荐);
Ts = 50e-6;        % 控制/仿真采样时间 (s)

% --- 3. SMO 观测器增益计算 ---
% (这里可以添加您之前计算好的SMO增益 A, B, k 等)
% 示例：假设最高转速为 6000 RPM
max_rpm_smo = 6000;
max_we_smo = (max_rpm_smo * 2*pi/60) * Pn;
max_emf = max_we_smo * psi_f;


Ad = Ts / Ld;
Bd = 1-(Rs*Ts/Ld);

Aq = Ts / Lq;
Bq = 1-(Rs*Ts/Lq);


% --- 4. PLL 控制器参数自动整定 ---
fprintf('Tuning PLL PI parameters...\n');

% --- PLL 设计目标 (您可以在这里修改) ---
target_RPM = 3000;   % [可调] PLL 性能最优化的目标机械转速 (RPM)
pll_f_n = 150;       % [可调] PLL 期望的闭环带宽 (Hz)
pll_zeta = 0.707;    % [可调] PLL 期望的阻尼比 (0.707为最佳值)

% --- 自动计算过程 ---
% 1. 计算目标转速下的电角速度 (we)
% target_wm = target_RPM * (2*pi/60);      % 目标机械角速度 (rad/s)
% target_we = target_wm * Pn;              % 目标电角速度 (rad/s)
% 
% % 2. 计算该转速下的等效增益 (Vq)
% Vq_pll = target_we * psi_f;
% if Vq_pll == 0; Vq_pll = 1e-6; end % 防止在0转速下分母为0
% 
% % 3. 计算期望带宽的角频率 (wn)
% pll_wn = 2 * pi * pll_f_n; % 带宽 (rad/s)
% 
% % 4. 计算连续域PI参数 (gamma_p, gamma_i)
% gamma_p = (2 * pll_zeta * pll_wn) / Vq_pll;
% gamma_i = pll_wn^2 / Vq_pll;
% 
% % 5. 计算离散域PI参数 (Kp_pll, Ki_pll)
% Kp_pll = gamma_p;
% Ki_pll = gamma_i * Ts;

Kp_pll = 200;      % [关键] 从一个很小的值开始
Ki_pll = 50;   % [关键] 积分增益也相应减小

k_smo = 8; % 滑模增益, d轴和q轴使用相同值
k_sat = 1; % 饱和增益

lpf_tc = 0.0016; % 对应约100Hz的截止频率，提供强力滤波


% --- 5. FOC 电流环 PI 参数 (零点极点对消法) ---
fprintf('Tuning FOC Current PI parameters...\n');

% [可调] 设置你期望的电流环带宽 (Hz)。
% 推荐值为采样率的 1/10 到 1/20。
% 采样率 = 10k Hz，所以 500 Hz 到 1000 Hz 是一个很好的起点。
f_bw_current = 200;  % 让我们从 1000 Hz 开始

% --- 自动计算 ---
bw_current = 2 * pi * f_bw_current;  % 转换为 rad/s

% Kp 和 Ki 是连续域增益。
% 你的 Simulink PI(z) 模块是离散的，但它内部
% 会自动处理 (P + I*Ts/(z-1))，所以我们只需要提供连续增益。
Kp_current = Ld * bw_current;  % Ld = 2.5E-5
Ki_current = Rs * bw_current;  % Rs = 0.14

fprintf('  Current Loop Ts    : %.5f s (%.1f kHz)\n', Ts, 1/(Ts*1000));
fprintf('  Current Loop Kp (L*bw): %.4f\n', Kp_current);
fprintf('  Current Loop Ki (R*bw): %.4f\n', Ki_current);