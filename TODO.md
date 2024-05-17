**TODO**

1. modify `gyro-acce_sensor.ino` into a function that can be called in `main()`.

2. start to implement `main()` involving: 
- reading two voltage signals from the EMG sensor;
- reading the displacements from the IMU sensors;
- calculate the angle of the elbow via data from IMU sensors;
- output the tuple containing the EMG and IMU data into a .scv file.