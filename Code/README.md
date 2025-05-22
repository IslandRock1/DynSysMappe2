
# Car Simulation for Adaptive Cruise Control (ACC)

## Overview

This project simulates the behavior of two or more cars, using a custom ACC regulator. The primary goal is to test different ACC regulators in order to achieve the best results when it comes to safety and comfort.

## File Structure

For this project there are two helper files, Car.py and DynSysPID.py. These are responsable for the simulation and interaction. The file main.py will set up the enviornment, letting the user easily testing out different situations and combinations.

### DynSysPID.py

DynSysPID.py is a simple implementation of PID, letting the user decide Kp, Ki, Kd, the max actuating signal, and the delta time (dt). Letting the user decide the dt makes faster than real time simulation possible, which is really important to effectly get results.

### Car.py

The Car.py file lets the user decide different parameters for the simulated car, then use a state-space model to simulate the car. There is a built-in regulator combining two PID-regulators, but this is easy to change, giving great flexibility in order to test multiple different regulators in order to achieve the best one.

### main.py

main.py is responsible for setting up the enviornment. This includes placing the cars, deciding goals such as the distance between the cars and the desired speed. Then plotting the results in a easy to read format.