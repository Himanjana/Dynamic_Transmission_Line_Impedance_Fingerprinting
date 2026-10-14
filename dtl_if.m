clear;
clc; 
close all;

% System Parameters
fs = 100e3;             %sampling rate
dt = 1/fs;              
duration = 0.2;         % 200ms window
t = 0:dt:duration-dt;   
N = length(t);          

% Grid Parameters
f_grid = 50;            
V_nom = 11000;          % Peak L-G voltage
I_nom = 200;            

% Nominal 50Hz telemetry
v_fund = V_nom * sin(2*pi*f_grid*t);
i_fund = I_nom * sin(2*pi*f_grid*t - deg2rad(30)); 

% Probing Signal Generation 
% Fixed seed for reproducible key sequence
rng(42); 
seed_bits = [1 0 1 1 0 1 0 0 1 1]; 
chip_noise = randn(1, length(seed_bits)); 
key = double((seed_bits + chip_noise) > 0.5);

% Frequency Hopping setup
freq_pool = [2000, 3500, 5000, 7500, 9000]; 
hop_time = 0.04;                      % Hop every 40ms
num_hops = round(duration / hop_time);
samples_per_hop = round(hop_time * fs);

f_carrier = zeros(1, N);
for h = 1:num_hops
    idx = mod(key(mod(h-1, length(key))+1) + h, length(freq_pool)) + 1;
    i1 = (h-1)*samples_per_hop + 1;
    i2 = min(h*samples_per_hop, N);
    f_carrier(i1:i2) = freq_pool(idx);
end

% Continuous phase integration 
phase_vec = 2 * pi * cumsum(f_carrier) * dt;

% alpha kept low to avoid triggering relay over-voltage
alpha = 0.005;          % 0.5% of nominal V
phi_ref = deg2rad(45);   
s_probe = alpha * V_nom * sin(phase_vec + phi_ref);

% grid + probe signal
v_tx = v_fund + s_probe;

% 50km Overhead Line
L_km = 50;              
R_val = 0.125 * L_km;       % Ohms/km
L_val = 1.05e-3 * L_km;     % H/km
C_val = 9.5e-9 * L_km;      % F/km

% Thermal variation factor (sag/temp rise)
temp_factor = 1.05;     
L_line = L_val * temp_factor;
C_line = C_val / temp_factor;
R_line = R_val * temp_factor;

% Line TF H(s)
sys_line = tf(1, [L_line*C_line, R_line*C_line, 1]);
[h_resp, ~] = lsim(sys_line, v_tx, t);
v_rec = h_resp';

% PMU sensor measurement noise (~50 dB SNR)
noise_pwr = (0.002 * V_nom)^2; 
v_meas = v_rec + sqrt(noise_pwr) * randn(1, N);

% Line Z disturbance window (80ms to 160ms)
ENABLE_DISTURBANCE = 1; 

if ENABLE_DISTURBANCE == 1
    dist_idx = (t >= 0.08) & (t <= 0.16);
    v_meas(dist_idx) = v_meas(dist_idx) + 0.10 * V_nom * sin(2*pi*f_grid*t(dist_idx));
end

% Dual-Band Receiver & Residual Anomaly Detector
% HPF to extract HF probe (1000Hz)
[b_hp, a_hp] = butter(4, 1000/(fs/2), 'high');
y_meas_hf = filter(b_hp, a_hp, v_meas);

% Expected HF channel response from baseline model
u_injected_hp = filter(b_hp, a_hp, s_probe);
[y_exp_sim, ~] = lsim(sys_line, u_injected_hp, t);
y_exp_hf = y_exp_sim';

% LPF to extract 50Hz fundamental (500Hz cutoff)
[b_lp, a_lp] = butter(4, 500/(fs/2), 'low');
v_meas_lf = filter(b_lp, a_lp, v_meas);
v_exp_lf = filter(b_lp, a_lp, v_fund);

% Sliding window error metric R (5ms window)
win_len = 500; 
R_metric = zeros(1, N);

for k = win_len:N
    w_idx = (k - win_len + 1):k;
    
    hf_err = norm(y_meas_hf(w_idx) - y_exp_hf(w_idx)) / sqrt(win_len);
    lf_err = norm(v_meas_lf(w_idx) - v_exp_lf(w_idx)) / sqrt(win_len);
    
    R_metric(k) = (hf_err + lf_err) / V_nom;
end

% Mute initial filter transients (first 10ms)
trans_samples = round(0.01 * fs);
R_metric(1:trans_samples) = 0;

% Threshold
tau = 0.04; 
flag_detect = R_metric > tau;

% Plots
figure('Name', 'Line Health Diagnostics', 'Color', 'w');

% Plot 1: Active Probing Signal
subplot(3, 1, 1);
plot(t*1000, s_probe, 'b', 'LineWidth', 1.2);
title('Injected Probing Signal s_{probe}(t)');
xlabel('Time (ms)'); ylabel('Voltage (V)');
grid on; xlim([0 duration*1000]);

% Plot 2: Received Substation Voltage
subplot(3, 1, 2);
plot(t*1000, v_rec/1000, 'k--', 'LineWidth', 1); hold on;
plot(t*1000, v_meas/1000, 'r', 'LineWidth', 1.2);
title('Substation Telemetry (Baseline vs Line Disturbance)');
legend('Baseline Waveform', 'Measured Waveform');
xlabel('Time (ms)'); ylabel('Voltage (kV)');
grid on; xlim([0 duration*1000]);

% Plot 3: Residual Metric R & Detection Window
subplot(3, 1, 3);
plot(t*1000, R_metric, 'm', 'LineWidth', 1.5); hold on;
yline(tau, 'r--', 'Threshold (\tau)', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
fill_x = [t*1000, fliplr(t*1000)];
fill_y = [flag_detect * max(R_metric) * 1.2, zeros(1, N)];
patch(fill_x, fill_y, 'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none');
title('Impedance Health Metric R & Detection Window');
legend('Metric R', 'Threshold \tau', 'Disturbance Region');
xlabel('Time (ms)'); ylabel('Normalized Metric');
grid on; xlim([0 duration*1000]);


