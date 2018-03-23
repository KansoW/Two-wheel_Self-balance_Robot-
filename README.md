# **Two-wheel Self_balance Robot with Arduino** 
### **General Idea**
##### This is a project that combines classical ideas from other projects. 
 1. The inverse pendulum problem that is often seen in the dynamics and control peojects or courses.
 2. The Arduino car driving project that is often used as a startup proejcts for learning Arduino.
 3. The IMU (_Inertial Measurement Unit_) that is widely used for balancing projects.
##### This project itself is fairly straight forward. There are many research papers and technical blog found online that tackled this problem successfully. I am humble and glad that I could get some help from these resources.

### Materials 
#### Electrical
1. Arduino UNO V3
2. DF Robot L298N Motor Driver
3. Pololu 20.4:1 Metal Gearmotor 25Dx50L mm LP 12V with 48 CPR Encoder
4. MPU6050 6-axis IMU (accelerometer + gyro)
5. 12V 1600 mAh NiMh Battery
6. Mini breadborad
7. Jumper wires
8. Power cable toggle switch
#### Mechanical
1. Pololu Acrylic Chassis Borads
2. DF Robot Metal Alluminum Standoffs
3. Pololu 80mm dia Wheels
4. Pololu 25mm Metal Gear Brackets
5. Polou 4mm Motor Shaft Adapters
4. 200g Metal Weight
### Balancing Control Theory
##### PID
##### LQR

### Code and Algorithm
##### The main Arduino IDE code is called **ref_test.ino**(as of right now).
##### All the needed libraries are also in the directory called **code/libraries**

### Progress
###### As of right now, I got the robot balancing with simple PID control. Here are some videos showing the results:
###### ![Simple PID_1](https://github.com/KansoW/Two-wheel_Self-balance_Robot-/blob/master/video/VID_20180314_053201.mp4)
###### ![Simple PID_2](https://github.com/KansoW/Two-wheel_Self-balance_Robot-/blob/master/video/VID_20180314_053236.mp4)
###### (I will update pictures later...)



### Refrence and Sources
##### 1. SUHARDI , et al. “[TWO-WHEELED BALANCING ROBOT CONTROLLER DESIGNED USING PID.](https://pdfs.semanticscholar.org/5a5b/3b44c6456ae231162ce013a8e493dc1bc6db.pdf)” Faculty of Electrical and Electronic Engineering Universiti Tun Hussein Onn Malaysia, Jan. 2015, pp. 1–39.
##### 2. Stanford University EE363. “[Linear Quadratic Regulator: Discrete-Time Finite Horizon.](https://stanford.edu/class/ee363/lectures/dlqr.pdf)” Linear Quadratic Regulator: Discrete-Time Finite Horizon, Jan. 2009, pp. 1–32.()
##### 3. CDS 110b. “[Lecture 2 – LQR Control](https://www.cds.caltech.edu/~murray/courses/cds110/wi06/lqr.pdf).” CALIFORNIA INSTITUTE OF TECHNOLOGY Control and Dynamical Systems, 11 Jan. 2006, pp. 1–14.
##### 4. [DF Robot MD1.3 2A Dual Motor Controller SKU DRI0002](https://www.dfrobot.com/wiki/index.php/MD1.3_2A_Dual_Motor_Controller_SKU_DRI0002)
##### 5. [Pololu 20.4:1 Metal Gearmotor 25Dx50L mm LP 12V with 48 CPR Encoder](https://www.pololu.com/product/3263)
##### 6. [Instructables ARDUINO SELF-BALANCING ROBOT](https://www.instructables.com/id/Arduino-Self-Balancing-Robot-1/)
##### 7. [MPU-6050 auto-calibration](http://42bots.com/tutorials/arduino-script-for-mpu-6050-auto-calibration/)
##### 8. [HC-06 Bluetooth module datasheet and configuration with Arduino
](http://42bots.com/tutorials/hc-06-bluetooth-module-datasheet-and-configuration-with-arduino/)
##### 9. [WOLFRAM BLOG Stabilized Inverted Pendulum](http://blog.wolfram.com/2011/01/19/stabilized-inverted-pendulum/)
##### 10. [Stages of development of the robot-balancer](http://spin7ion.ru/ru/blog/balancerBuildSteps)

