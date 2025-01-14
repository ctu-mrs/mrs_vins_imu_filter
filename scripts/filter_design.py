#!/bin/python3

## Python script that generates the Butterworth filter based on specified parameters and prints the nominator/denominator filter parameters that can be directly copied into the imu_filter node config

import scipy.signal

# Order of filter (higher order means higher slope but larger delay)
order = 1

# Critical frequency of the filter [Hz]
# For a Butterworth filter, this is the point at which the gain drops to 1/sqrt(2) that of the passband (the “-3 dB point”).
threshold_freq = 10

# Sampling frequency of the data [Hz] (find out using e.g. 'rostopic hz' on the IMU topic)
sampling_freq = 200

[b, a] = scipy.signal.butter(order, threshold_freq, btype='low', analog=False, output='ba', fs=sampling_freq)

print("a: [" + str(a[0]) + ", " + str(a[1]) + "]")
print("b: [" + str(b[0]) + ", " + str(b[1]) + "]")
