# crazyflie_test
##Control crazyflie with gesture

##Requirements for only hovering:
<br/>[crazyflie_ros](https://www.github.com/whoenig/crazyflie_ros)
<br/>And this repo 
##Requirements for gesture control:
<br/>All the requirements for only hovering
###Drivers and patches:
<br/>openni 1.5.x (including openni itself, NITE 1.5.x, Patch for primesense repo: [SensorKinect](https://github.com/avin2/SensorKinect))
###Repos:
<br/>Modified openni_tracker repo: [mod_openni_tracker](https://github.com/hzheng40/mod_openni_tracker)
<br/>TF data processing repo: [tf_processing](https://github.com/hzheng40/learning_tf)
<br/>clone all the repos including this one and put them in your workspace
##To run the project:
<br/>1. catkin_make
<br/>2. roslaunch openni_lauch openni_launch.launch
<br/>3. rosrun mod_openni_tracker mod_openni_tracker
<br/>4. rosrun learning_tf tf_test
<br/>5. roslaunch crazyflie_driver crazyflie_server.launch
<br/>6. roslaunch crazyflie_driver crazyflie_add.launch uri:=//0/80/250k (double check the uri for your crazyflie but it should be this one if you're only running one drone)
###IMPORTANT: Before running the main program flying the drone, make sure a user is in frame and do the psi pose to start calibration. Make sure the tracker started tracking and tf_test is publishing and updating data
<br/>7. rosrun crazyfie_test crazyflie_pressure (for pressure controlled thrust)
<br/>OR: rosrun crazyflie_test crazyflie_test (for constant thrust)

