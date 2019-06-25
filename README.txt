This is a project I've been working on for over a year now. This self-balancing robot uses a MPU-6050 IMU to sense orientaion, and
a complimentary filter to interpret that orientation over time. The robot uses two 30:1 geared DC motors with quadrature encoders
to propel itself. Two nested PID loops balance the robot, the outermost loop uses the velocity calculated from the encoders to 
control the innermost PID loop and adjusts the angle of the robot to bring the velocity to 0, while the innermost control loop
controls the motor output to bring the angle of the robot to 0.  