# pick_and_place

.bashrc configured for baxter

roscore

baxter enable:

rosrun baxter_tools enable_robot.py -e

roslaunch openni_launch openni.launch

rosrun image_view image_view image:=/camera/rgb/image_raw

roslaunch teleop_interface teleop_calibration.launch

roslaunch teleop_interface teleop_interface.launch

rosrun finger_sensor sensor.py

for visual: rosrun finger_sensor sensor_visual.py

rosrun finger_sensor stacking_blocks.py

rosrun keyboard keyboard

calibration:

in keyboard window hit c
