# CarND-Path-Planning-Project
The steps of this project is follow:
1. Check which lane is safe based on sensor_fusion data;
2. Choice the best lane to change;
3. Creat the path point used spline;
4. Control speed change used PD control;

## Check which lane is safe based on sensor_fusion data
1. I check the other traffics in every lane. 
2. I check the traffic is located which lane. The code can be found the line133 ~ line134 in main.cpp.
3. I predicted the next position of every traffic by using the traffic speed and the last path point of the self_car predicted.The code can be found the line137 ~ line142 in main.cpp.
4. I check the predict s of the traffic is located between -8 and 30 meters than the position of the self_car in Frenet. So that I can know which lane is safe.Refered to  line144 ~ line156 in main.cpp.
5. I check the predict s of the traffic is located between -8 and 20 meters than the position of the self_car in Frenet coordinate system. And the traffic is also located in the middle lane. I get this status which named ```middle_collision``` in order to make the car-self can change from lane0 to lane2 or from lane2 to lane0. Refered to  line158 ~ line162 in main.cpp.

## Choice the best lane to change
 I classified three kinds of situation as follow:
1. When current lane is safe, the car should keep the current lane;
2. When all three lanes are unsafe, the car should keep the current lane and the speed should be same with the front car.
3. When current lane is unsafe, the car can choice the other lanes which is safe; But the car can change to lane beyond middle lane only when the status of the ```middle_pass_flag``` is safe.
4. In order to prevent greater jitter, it is necessary to maintain the same lane for a period of time. I select 20 cycles after test many times. The code refered to  line286 ~ line306 in main.cpp.

## Creat the path point used spline;
1. I have learned how to creat the smooth path used spline from lesson;
2. After before two steps process, I can get next lane which the car would choice.
3. I get the last two points of the last path which generates. And I get three points which locate the best lane. The s coordinate of three points is (30,60,90) in Frenet coordinate system.

## Control speed change used PD control
1. I set one value named target-speed which means the car should reach target_speed. And I set one named speed_current which means sending speed command at the moment.
2. The speed_current can reach target-speed by increase or reduce a smaller value. I used this way to avoid the big acceleration. 
3. The code refered to  line332 ~ line343 in main.cpp.
