
# VINS IMU filter

Provides filtering of IMU data to be used in VIO/VINS. Provides Butterworth low-pass filter and notch filters, configured separately for accelerometer and gyroscope data. Provides joining of separate accelerometer and gyroscope messages in case of the RealSense T256 IMU by copying the latest accelerometer message.

## Usage

To use with the ICM-42688 IMU:
```
roslaunch mrs_vins_imu_filter filter_icm_42688.launch
```

To use with the RealSense T265 IMU:
```
roslaunch mrs_vins_imu_filter filter_t265.launch
```

## Configuration
Edit / create configs in `config/` folder.
