
Readme for carla data config

Carla_01: corresponding to the carla_dataset_5

carla_dataset_5 imu_paramter is (Carla_01 (1km))
acc_n: 1.1089927780327790e-02           # accelerometer measurement noise standard deviation. #0.04
gyr_n: 1.0140952581436385e-02          # gyroscope measurement noise standard deviation.     #0.008
acc_w: 2.0583587465800817e-05        # accelerometer bias random work noise standard deviation.  #0.00004
gyr_w: 9.1355383994881894e-05  

the image resolution is 
width: 1382
height: 512


Carla_02(Carla_02 2km) and Carla_03(Carla_03 3km) corresponding to the carla_dataset_8

rosbag play -s 3 -u 200 carla_dataset_8.bag  or rosbag play -s 3 -u 250 carla_dataset_8.bag

imu parameter:
acc_n: 1.1089927780327790e-02           # accelerometer measurement noise standard deviation. #0.04
gyr_n: 1.0140952581436385e-02          # gyroscope measurement noise standard deviation.     #0.008
acc_w: 2.0583587465800817e-05        # accelerometer bias random work noise standard deviation.  #0.00004
gyr_w: 9.1355383994881894e-05 
the image resolution is 
width: 960
height: 600

initial R t
   data: [-0.0014,    1.0,   0.007,
         -1.0,    -0.0014,   0.0,
          0.0,    -0.007,   1.0]
   data: [15.5, -77.80, -2.1]



Carla_03

rosbag play -s 310 -u 300 carla_dataset_8.bag

imu parameter:
acc_n: 1.1089927780327790e-02           # accelerometer measurement noise standard deviation. #0.04
gyr_n: 1.0140952581436385e-02          # gyroscope measurement noise standard deviation.     #0.008
acc_w: 2.0583587465800817e-05        # accelerometer bias random work noise standard deviation.  #0.00004
gyr_w: 9.1355383994881894e-05 
the image resolution is 
width: 960
height: 600

initial R t
   data: [-0.984017359238086,    0.0210114682972551,    0.176828603223850,
           -0.0213580449804165,    -0.999771889337277,   -5.66178923439933e-05,
           0.176787077108931,     -0.00383242625038918,   0.984241658271136]   # begin 310
   data: [31.86, 138.20, -9.73]  #begin 310s
           
           


