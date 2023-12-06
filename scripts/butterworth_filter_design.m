%% Matlab script for butterworth low-pass filter design
fs = 400; % sampling frequency
fc = 10; % cutoff frequency
n = 1; % filter order
Wn = fc/(fs/2);
[b,a] = butter(n, Wn, 'low')
