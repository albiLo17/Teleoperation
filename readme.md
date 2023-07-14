# Franka Teleoperation with Oculus

Teleoperate the Franka through Oculus Quest.

## Running the code

1. Initialize the robot (Baer).

   ```shell
   roslaunch teleoperation start_robot.launch
   rosrun teleoperation switch_controllers.py

2. Start the VR interface.
   ```shell
   rosrun teleoperation oculus_data_collection.py

## Useful Links

Here are some useful links related to the project:

- [VR Init](https://github.com/rail-berkeley/oculus_reader): Github repo used to initialize the VR interface.



