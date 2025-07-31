# Firmware for Fredo1 3-DoF Manipulator
This is a repo storing the firmware of [FREDO1](https://github.com/Dynamics-Operator-Foundry/Fullstack-RoboticsBootcamp/tree/master/fredo1). Within the firmware, we exploited ```ARDUINO + SERIAL``` and ```socket``` to achieve communication amongst each executables.

## Tree
- ```fo1```
    - ```fo1.ino```: low-level communication for pulse sending and signal acquisition.
- ```fo1_serial/src```
    - ```serial_link.cpp```: frontal communication via serial with ```ARDUINO```.
    - ```serial_com.cpp```: handling low raw signals and readable estimation and control inputs.
    - ```com_util.cpp```: communication libraries similar to ```ROS```.
    - ```ctrl_test.cpp```: angle testing.

## Prerequisites
Please refer to [here](./install.md).

## Usage
- To bootload the code into ```ARDUINO```, do 
    ```
    ./bootload.sh fo1
    ```
- To launch 
    ```
    ./launch.sh TWIN # TWIN mode for visualizing
        || 
    ./launch.sh CTRL # CTRL mode for controlling
    ```